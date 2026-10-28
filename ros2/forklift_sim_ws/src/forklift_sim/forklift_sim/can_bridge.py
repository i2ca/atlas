"""Ponte vcan0 -> Gazebo para a empilhadeira Hyster.

Le do CAN o joystick da garra (0x0CFFC0EC, o mesmo frame que o forklift_driver
do astro manda) e move as juntas do mastro e da garra no Gazebo, como os
hidraulicos da empilhadeira real: a alavanca define a velocidade, sem
realimentacao de posicao. Opcionalmente obedece a falha "garra desconectada" da
VCU simulada (0x18FF1027, do forklift_can_simulator).

A base anda pela velocidade e pela marcha que a empilhadeira (ou o
forklift_can_simulator) publica no CAN -- CCVS 18FEF127 e ETC2 18F00527, as mesmas que
o vehicle_driver le'. A direcao vem do SteeringAngle da placa de interceptacao
(18FF51F0); sem ele, a guinada vem do /cmd_vel (angular.z). O /cmd_vel inteiro so'
vale quando nao ha velocidade no CAN. As rodas e
a direcao traseira giram junto, so para a visualizacao.

TEMPO REAL: a velocidade do CAN e' em m por segundo REAL (o forklift_can_simulator e o astro rodam
no relogio da maquina), mas o Gazebo integra no relogio da SIMULACAO. Com o Gazebo abaixo do tempo
real (medido em 22/10 com a janela aberta: 0,65 a 0,8), a empilhadeira andava ~1,5x menos no Gazebo
do que a odometria do astro dizia -- "anda pouco e no mapa anda muito" -- e a localizacao escapava.
Por isso a base e as rodas recebem v / fator_de_tempo_real (tempo da simulacao do /world/<mundo>/stats do proprio Gazebo
-- o /clock do ROS nao chega aqui -- contra o relogio real, numa janela de ~1 s):
por segundo real, ela anda no Gazebo o que o CAN diz. compensate_rtf:=false desliga.

CORPO (gz_body.py, body_physics:=true): a base nao escala parede/fardo (nivelamento), para ao encostar num
obstaculo e avisa o forklift_can_simulator pelo [SIM] 18FF53FE (sentido bloqueado + velocidade medida no
Gazebo), e a garra prende/solta as unidades de celulose com fisica (pulp_dyn_<n>) ao fechar/abrir nelas.
"""

import math
import socket
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from forklift_sim.gz_body import GzBody

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FMT = '=IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

ID_CLAMP_AXES = 0x0CFFC0EC
ID_CLAMP_BUTTON = 0x0CFFC1EC
ID_VCU_STATUS = 0x18FF1027
ID_CCVS = 0x18FEF127        # velocidade (bytes 2-3, 1/256 km/h) e freio de mao
ID_ETC2 = 0x18F00527        # marcha atual (byte 4, bruto - 125: 1, 0, -1)
CCVS_FACTOR = 0.001085069   # m/s por bit
ID_INTERCEPT_STATUS = 0x18FF51F0  # [SIM] placa de interceptacao: SteeringAngle (bytes 3-4, 0.0001 rad, com sinal)
CAN_SPEED_TIMEOUT = 0.5
ID_GAZEBO_FEEDBACK = 0x18FF53FE   # [SIM] Gazebo -> simulador: byte 1 bit 0 frente bloqueada, bit 1 re';
                                  # bytes 3-4 velocidade medida (0.001 m/s, com sinal)

# limites das juntas (iguais ao URDF)
CARRIAGE_MIN, CARRIAGE_MAX = -0.25, 2.3         # carriage_joint
MAST_MIN, MAST_MAX = 0.0, 2.65                  # mast_lift_joint
SHIFT_MIN, SHIFT_MAX = -0.3, 0.3                # clamp_base_joint (deslocamento lateral)
GRIP_MIN, GRIP_MAX = 0.0, 0.45                  # clamp_1/2_joint: 0 = aberta, + = fechando

FUNCTIONS = ('lift', 'sideshift', 'clamp', 'none')


def decode_axis(word):
    """Eixo de joystick J1939: bits 2-3 negativo, 4-5 positivo, 6-15 alavanca (0,1 %)."""
    negative = (word >> 2) & 0x3
    positive = (word >> 4) & 0x3
    lever = ((word >> 6) & 0x3FF) * 0.1
    if lever > 100.0:
        lever = 100.0
    if negative == 1 and positive != 1:
        return -lever / 100.0
    if positive == 1 and negative != 1:
        return lever / 100.0
    return 0.0


def clamp(value, low, high):
    return max(low, min(high, value))


class CanBridge(Node):

    def __init__(self):
        super().__init__('can_bridge')
        p = self.declare_parameter
        self.iface = p('can_interface', 'vcan0').value
        self.command_timeout = p('command_timeout', 0.3).value
        self.respect_vcu_fault = p('respect_vcu_fault', True).value
        self.axis_function = []
        self.axis_sign = []
        defaults = ('lift', 'sideshift', 'clamp', 'none')
        for i in range(4):
            function = p(f'axis{i + 1}_function', defaults[i]).value
            if function not in FUNCTIONS:
                raise ValueError(f'axis{i + 1}_function invalido: {function}')
            self.axis_function.append(function)
            self.axis_sign.append(p(f'axis{i + 1}_sign', 1.0).value)
        self.rate = {
            'lift': p('lift_rate', 0.4).value,
            'sideshift': p('sideshift_rate', 0.1).value,
            'clamp': p('clamp_rate', 0.15).value,
        }
        self.wheel_radius = p('wheel_radius', 0.405).value
        self.wheelbase = p('wheelbase', 2.235).value
        self.max_steer = p('max_steer', 0.6).value
        self.steer_sign = p('steer_sign', 1.0).value
        self.drive_sign = p('drive_sign', 1.0).value
        self.cmd_vel_timeout = p('cmd_vel_timeout', 0.5).value
        self.compensate_rtf = p('compensate_rtf', True).value
        self.rtf_window = p('rtf_window', 1.0).value       # s (reais) da janela de medida
        self.rtf_min = p('rtf_min', 0.2).value             # abaixo disso nao compensa mais (travadas)
        self.clock_samples = []                            # (relogio real, relogio da simulacao)
        self.rtf = 1.0
        self.body_physics = p('body_physics', True).value          # pega dos fardos
        self.base_physics = p('base_physics', False).value         # nivelamento/bloqueio da base (experimental)

        # can_interface "none": sem CAN, so /cmd_vel (para testar o Gazebo sem o vcan0)
        self.sock = None
        if self.iface != 'none':
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            try:
                self.sock.bind((self.iface,))
            except OSError as error:
                raise SystemExit(
                    f'can_bridge: nao abriu {self.iface} ({error}). Para criar o vcan0: sudo modprobe vcan'
                    ' && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0')
            self.sock.setblocking(False)

        self.axes = [0.0] * 4
        self.button = 0
        self.last_axes_rx = 0.0
        self.vcu_seen = False
        self.vcu_clamp_fault = False
        self.can_speed = 0.0
        self.can_gear = 0
        self.last_ccvs_rx = 0.0
        self.can_phi = 0.0
        self.last_steer_rx = 0.0

        # estado "hidraulico": elevacao total (carro + mastro), deslocamento lateral, fechamento
        self.lift = 0.0
        self.shift = 0.0
        self.grip = 0.0

        self.cmd_vel = Twist()
        self.last_cmd_vel = 0.0

        self.mast_pub = self.create_publisher(Float64MultiArray, '/mast_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/steer_controller/commands', 10)
        self.drive_pub = self.create_publisher(Float64MultiArray, '/drive_controller/commands', 10)
        self.base_pub = self.create_publisher(Twist, '/model/hyster/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_handler, 10)
        self.start_stats_listener()
        self.body = None
        if self.body_physics:
            self.body = GzBody(self.get_logger())
            self.body.start()
        self.last_feedback = 0.0

        self.last_step = time.monotonic()
        self.create_timer(0.005, self.poll_can, clock=rclpy.clock.Clock())
        self.create_timer(0.02, self.step, clock=rclpy.clock.Clock())
        self.create_timer(1.0, self.report, clock=rclpy.clock.Clock())

        mapping = ', '.join(f'eixo {i + 1}={f}' for i, f in enumerate(self.axis_function))
        self.get_logger().info(f'Lendo {self.iface}; {mapping}')

    def start_stats_listener(self):
        """Assina o /world/<mundo>/stats do Gazebo (gz.transport) numa thread propria."""
        if not self.compensate_rtf:
            return
        try:
            from forklift_sim.gz_compat import Node as GzNode, msg
            WorldStatistics = msg('world_stats', 'WorldStatistics')
        except ImportError as error:
            self.get_logger().warning(f'sem gz.transport ({error}): base sem compensacao de tempo real')
            return

        def wait_and_subscribe():
            self.gz_node = GzNode()
            while rclpy.ok():
                topics = [t for t in self.gz_node.topic_list() if t.startswith('/world/') and t.endswith('/stats')]
                if topics:
                    self.gz_node.subscribe(WorldStatistics, topics[0], self.stats_handler)
                    self.get_logger().info(f'tempo da simulacao por {topics[0]}')
                    return
                time.sleep(1.0)
        threading.Thread(target=wait_and_subscribe, daemon=True).start()

    def stats_handler(self, msg):
        now = time.monotonic()
        self.clock_samples.append((now, msg.sim_time.sec + msg.sim_time.nsec * 1e-9))
        while len(self.clock_samples) > 2 and now - self.clock_samples[0][0] > self.rtf_window:
            self.clock_samples.pop(0)
        (w0, s0), (w1, s1) = self.clock_samples[0], self.clock_samples[-1]
        if w1 - w0 > 0.2 and s1 >= s0:
            self.rtf = (s1 - s0) / (w1 - w0)
        elif s1 < s0:   # simulacao reiniciada
            self.clock_samples = self.clock_samples[-1:]

    def time_scale(self):
        """Fator para converter m/s reais (CAN) em m/s da simulacao."""
        if not self.compensate_rtf or time.monotonic() - (self.clock_samples[-1][0] if self.clock_samples else 0.0) > 1.0:
            return 1.0
        return 1.0 / max(self.rtf, self.rtf_min)

    def cmd_vel_handler(self, msg):
        self.cmd_vel = msg
        self.last_cmd_vel = time.monotonic()

    def poll_can(self):
        while self.sock is not None:
            try:
                frame = self.sock.recv(CAN_FRAME_SIZE)
            except BlockingIOError:
                return
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
            if not can_id & CAN_EFF_FLAG:
                continue
            can_id &= CAN_EFF_MASK
            if can_id == ID_CLAMP_AXES and dlc >= 8:
                words = struct.unpack('<4H', data)
                self.axes = [decode_axis(w) for w in words]
                self.last_axes_rx = time.monotonic()
            elif can_id == ID_CLAMP_BUTTON and dlc >= 1:
                self.button = (data[0] >> 6) & 0x3
            elif can_id == ID_VCU_STATUS and dlc >= 5:
                self.vcu_seen = True
                self.vcu_clamp_fault = bool((data[4] >> 2) & 0x1)
            elif can_id == ID_CCVS and dlc >= 3:
                self.can_speed = struct.unpack('<H', data[1:3])[0] * CCVS_FACTOR
                self.last_ccvs_rx = time.monotonic()
            elif can_id == ID_INTERCEPT_STATUS and dlc >= 4:
                self.can_phi = struct.unpack('<h', data[2:4])[0] * 0.0001
                self.last_steer_rx = time.monotonic()
            elif can_id == ID_ETC2 and dlc >= 4:
                gear = data[3] - 125
                self.can_gear = gear if gear in (-1, 0, 1) else 0

    def clamp_enabled(self):
        if time.monotonic() - self.last_axes_rx > self.command_timeout:
            return False
        if self.respect_vcu_fault and self.vcu_seen and self.vcu_clamp_fault:
            return False
        return True

    def step(self):
        now = time.monotonic()
        dt = min(now - self.last_step, 0.1)
        self.last_step = now

        if self.clamp_enabled():
            for value, function, sign in zip(self.axes, self.axis_function, self.axis_sign):
                if function == 'none' or value == 0.0:
                    continue
                delta = sign * value * self.rate[function] * dt
                if function == 'lift':
                    self.lift = clamp(self.lift + delta, CARRIAGE_MIN, CARRIAGE_MAX + MAST_MAX)
                elif function == 'sideshift':
                    self.shift = clamp(self.shift + delta, SHIFT_MIN, SHIFT_MAX)
                elif function == 'clamp':
                    self.grip = clamp(self.grip + delta, GRIP_MIN, GRIP_MAX)
        if self.body is not None:
            # o hidraulico trava ao encostar na carga: o fechamento nao passa do ponto de pega
            self.grip = self.body.grip_step(self.grip)

        # sobe primeiro o carro, depois o mastro
        carriage = clamp(self.lift, CARRIAGE_MIN, CARRIAGE_MAX)
        mast_lift = clamp(self.lift - CARRIAGE_MAX, MAST_MIN, MAST_MAX)
        mast = Float64MultiArray()
        # ordem do mast_controller: carriage, mast_lift, clamp_base, clamp_1, clamp_2
        mast.data = [carriage, mast_lift, self.shift, self.grip, self.grip]
        self.mast_pub.publish(mast)

        v = w = 0.0
        if now - self.last_cmd_vel < self.cmd_vel_timeout:
            v = self.cmd_vel.linear.x
            w = self.cmd_vel.angular.z
        if now - self.last_ccvs_rx < CAN_SPEED_TIMEOUT:
            # CCVS nao tem sinal: o sentido vem da marcha (com 0 ela esta' parada)
            v = self.can_speed * (self.can_gear if self.can_gear != 0 else 1)
        steer = 0.0
        if now - self.last_steer_rx < CAN_SPEED_TIMEOUT:
            # angulo de direcao lido do CAN: as rodas viram mesmo parada
            steer = clamp(self.can_phi, -self.max_steer, self.max_steer)
        elif abs(v) > 1e-3:
            steer = clamp(math.atan(w * self.wheelbase / v), -self.max_steer, self.max_steer)
        steer *= self.steer_sign
        self.steer_pub.publish(Float64MultiArray(data=[steer, steer]))
        # v (m/s reais) -> m/s da simulacao; a curvatura (direcao) nao muda
        v_sim = v * self.time_scale()
        w_sim = v_sim * math.tan(self.steer_sign * steer) / self.wheelbase if abs(v) > 1e-3 else 0.0

        # base_footprint: x para frente (lado da garra), origem no eixo dianteiro
        base = Twist()
        if self.body is not None and self.base_physics:
            (base.linear.x, base.linear.y, base.linear.z), (base.angular.x, base.angular.y, base.angular.z) = \
                self.body.base_command(v_sim, w_sim)
            if self.body.blocked and (v_sim > 0) == (self.body.blocked > 0):
                v_sim = 0.0
        else:
            base.linear.x = v_sim
            base.angular.z = w_sim
        self.base_pub.publish(base)
        wheel = self.drive_sign * v_sim / self.wheel_radius
        self.drive_pub.publish(Float64MultiArray(data=[wheel, wheel]))
        if self.body is not None and self.base_physics and self.sock is not None and now - self.last_feedback >= 0.05:
            self.last_feedback = now
            self.send_feedback()

    def send_feedback(self):
        flags = (1 if self.body.blocked > 0 else 0) | (2 if self.body.blocked < 0 else 0)
        v_real = self.body.v_meas / self.time_scale()
        data = struct.pack('<BBhxxxx', flags, 0, int(clamp(v_real * 1000.0, -32767, 32767)))
        try:
            self.sock.send(struct.pack(CAN_FRAME_FMT, ID_GAZEBO_FEEDBACK | CAN_EFF_FLAG, 8, data))
        except OSError:
            pass

    def report(self):
        age = time.monotonic() - self.last_axes_rx
        if self.last_axes_rx == 0.0:
            state = 'sem joystick no CAN'
        elif age > self.command_timeout:
            state = f'joystick parado ha {age:.1f} s'
        elif self.respect_vcu_fault and self.vcu_seen and self.vcu_clamp_fault:
            state = 'VCU com falha da garra'
        else:
            state = 'ok'
        axes = ' '.join(f'{a * 100:+4.0f}%' for a in self.axes)
        self.get_logger().info(
            f'garra {state} | eixos {axes} botao {self.button} | '
            f'elevacao {self.lift:.2f} m lateral {self.shift:+.2f} m fechamento {self.grip:.2f} m')
        if time.monotonic() - self.last_ccvs_rx < CAN_SPEED_TIMEOUT:
            self.get_logger().info(f'veiculo pelo CAN: {self.can_speed:.2f} m/s marcha {self.can_gear:+d} '
                                   f'direcao {math.degrees(self.can_phi):+.1f} graus | tempo real {self.rtf:.2f} '
                                   f'(base x{self.time_scale():.2f})')
        if self.body is not None:
            self.get_logger().info(self.body.status())


def main():
    rclpy.init()
    node = CanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
