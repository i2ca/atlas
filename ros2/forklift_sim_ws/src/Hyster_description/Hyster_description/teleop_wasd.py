import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Implement travel in metres, matching the joint limits in Hyster.xacro
CARRIAGE_MIN = -0.25
CARRIAGE_MAX = 2.3
MAST_MAX = 2.65
SHIFT_MAX = 0.3
GRIP_MAX = 0.45

HELP = """
Teleop WASD - Hyster
--------------------
   W        W / S : acelera para frente / para tras
 A S D      A / D : esterca para a esquerda / direita
            X     : centraliza a direcao
            ESPACO: para a tracao (a garra fica onde esta)

 T  Y  U    T / G : abre / fecha a garra
 G  H  J    Y / H : desloca a garra para a esquerda / direita
            U / J : sobe / desce a garra

            Q ou Ctrl-C: sair

Cada toque muda o comando em um degrau.
Mantenha este terminal em foco.
"""


class TeleopWasd(Node):

    def __init__(self):
        super().__init__('teleop_wasd')
        self.speed_step = self.declare_parameter('speed_step', 0.5).value
        self.max_speed = self.declare_parameter('max_speed', 3.0).value
        self.steer_step = self.declare_parameter('steer_step', 0.1).value
        self.max_steer = self.declare_parameter('max_steer', 0.6).value
        self.lift_step = self.declare_parameter('lift_step', 0.1).value
        self.shift_step = self.declare_parameter('shift_step', 0.05).value
        self.grip_step = self.declare_parameter('grip_step', 0.05).value
        self.speed = 0.0
        self.steer = 0.0
        self.lift = 0.0
        self.shift = 0.0
        self.grip = 0.0
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_targets', 10)
        self.grip_actual = None
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_timer(0.1, self.publish)

    def publish(self):
        msg = Twist()
        msg.linear.x = self.speed
        # The ackermann plugin reads angular.z as the steering angle and flips
        # it when reversing; undo that so A/D always turn the wheels the same way.
        msg.angular.z = self.steer if self.speed >= 0.0 else -self.steer
        self.pub.publish(msg)

        # Free lift first: the carriage runs up the inner mast before the mast extends
        targets = JointState()
        targets.name = [
            'carriage_joint', 'mast_lift_joint', 'clamp_base_joint',
            'clamp_1_joint', 'clamp_2_joint']
        targets.position = [
            min(self.lift, CARRIAGE_MAX), max(self.lift - CARRIAGE_MAX, 0.0), self.shift,
            self.grip, self.grip]
        self.joint_pub.publish(targets)

    def on_joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if 'clamp_1_joint' in positions and 'clamp_2_joint' in positions:
            self.grip_actual = min(positions['clamp_1_joint'], positions['clamp_2_joint'])

    def on_key(self, key):
        """Apply a key press; return False when the node should quit."""
        key = key.lower()
        if key == 'w':
            self.speed = min(self.speed + self.speed_step, self.max_speed)
        elif key == 's':
            self.speed = max(self.speed - self.speed_step, -self.max_speed)
        elif key == 'a':
            self.steer = min(self.steer + self.steer_step, self.max_steer)
        elif key == 'd':
            self.steer = max(self.steer - self.steer_step, -self.max_steer)
        elif key == 'x':
            self.steer = 0.0
        elif key == ' ':
            self.speed = 0.0
            self.steer = 0.0
        elif key == 'u':
            self.lift = min(self.lift + self.lift_step, CARRIAGE_MAX + MAST_MAX)
        elif key == 'j':
            self.lift = max(self.lift - self.lift_step, CARRIAGE_MIN)
        elif key == 'y':
            self.shift = min(self.shift + self.shift_step, SHIFT_MAX)
        elif key == 'h':
            self.shift = max(self.shift - self.shift_step, -SHIFT_MAX)
        elif key == 'g':
            self.grip = min(self.grip + self.grip_step, GRIP_MAX)
        elif key == 't':
            # Open from where the arms really are: against a load they stop short
            # of the commanded closing, and T should release on the first press.
            if self.grip_actual is not None:
                self.grip = min(self.grip, self.grip_actual)
            self.grip = max(self.grip - self.grip_step, 0.0)
        elif key in ('q', '\x03'):
            return False
        else:
            return True
        # Rounding keeps repeated steps from drifting off zero
        self.speed = round(self.speed, 3)
        self.steer = round(self.steer, 3)
        self.lift = round(self.lift, 3)
        self.shift = round(self.shift, 3)
        self.grip = round(self.grip, 3)
        sys.stdout.write(
            '\rvel %+.1f m/s | esterco %+.2f rad | altura %+.2f m | lateral %+.2f m | '
            'fechamento %.2f m   ' % (self.speed, self.steer, self.lift, self.shift, self.grip))
        sys.stdout.flush()
        return True


def main(args=None):
    if not sys.stdin.isatty():
        print('teleop_wasd precisa rodar em um terminal interativo.', file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = TeleopWasd()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            while running and select.select([sys.stdin], [], [], 0)[0]:
                running = node.on_key(sys.stdin.read(1))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
