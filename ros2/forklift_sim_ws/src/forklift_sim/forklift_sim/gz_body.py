"""Corpo da Hyster no Gazebo, para o can_bridge: base que nao "sobe" ao bater, parada por obstaculo e a garra
que pega as unidades de celulose.

A base anda pelo VelocityControl (velocidade imposta no referencial do corpo, ver o URDF). Isso tem dois efeitos
ruins quando ela encosta em algo:
  - o contato inclina o corpo e a velocidade "para frente" do corpo inclinado aponta para cima: ela escala a
    parede ou o fardo, como se nao tivesse peso;
  - a velocidade e' imposta: ela atravessa/empurra tudo, e o CAN continua dizendo que ela anda.
Aqui:
  - NIVELAMENTO: a velocidade pedida vira velocidade no MUNDO (horizontal, no rumo), mais uma correcao que
    leva a altura e a inclinacao de volta ao chao; so' depois volta para o referencial do corpo.
  - BLOQUEIO: com velocidade pedida e sem avanco, o sentido fica bloqueado: a base
    para naquele sentido e o can_bridge avisa o forklift_can_simulator no CAN (18FF53FE), que zera a
    velocidade -- como a empilhadeira real encostada numa parede. Desbloqueia ao inverter o sentido, ou
    tenta de novo depois de RETRY s. Fardo com fisica nao bloqueia: ele e' empurrado (anda).
  - GARRA: com uma unidade pulp_dyn_<n> entre os bracos, o fechamento para quando os dois bracos encostam
    nela (o hidraulico trava na carga) e a unidade e' presa no braco (DetachableJoint, /hyster/grab/<n>/attach);
    ao abrir, solta (.../detach). As de cima vao junto pelo atrito.

Poses: /world/<mundo>/dynamic_pose/info (gz.transport). Os links vem no referencial do modelo.
"""

import math
import threading
import time

import numpy as np

# Geometria da garra no referencial do link Base_das_garras_1 (Hyster.xacro: juntas clamp_1/2 e as caixas
# de colisao dos bracos). Fechar = clamp_1 anda +x, clamp_2 anda -x.
ARM1_FACE_OPEN = -1.74241 + 0.04009 + 0.04     # face interna do braco 1, garra aberta
ARM2_FACE_OPEN = -0.29241 - 0.03991 - 0.04     # face interna do braco 2
PAD_Y = (-0.165257 - 0.77719 - 0.7125, -0.165257 - 0.77719 + 0.7125)
PAD_Z = (-0.366 - 0.2535, -0.366 + 0.2535)
UNIT_SIZE = np.array([0.945, 1.39, 1.80])        # pulp_unit: L x W x H, origem no centro da base

ATTACH_TOL = 0.004      # m de folga para considerar os dois bracos encostados
ATTACH_HOLD = 0.4       # s com o comando no ponto de pega antes de prender (os bracos alcancam o comando)
RELEASE_OPEN = 0.02     # m abrindo alem do ponto de pega para soltar

# nivelamento
K_Z = 4.0               # 1/s
K_TILT = 4.0            # 1/s
MAX_VZ = 0.5
MAX_W_TILT = 0.6

# bloqueio
STALL_RATIO = 0.35      # avanco medido / pedido abaixo disso = sem avanco
STALL_TIME = 0.30       # s de simulacao
RETRY = 1.5             # s


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_to_axis_angle(r):
    """vetor de rotacao (eixo * angulo) de uma matriz de rotacao"""
    c = clampf((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(c)
    if angle < 1e-6:
        return np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2.0 * math.sin(angle))
    return axis * angle


def clampf(v, lo, hi):
    return max(lo, min(hi, v))


class GzBody:
    def __init__(self, logger, model='hyster', clamp_link='Base_das_garras_1'):
        self.log = logger
        self.model = model
        self.clamp_link = clamp_link
        self.lock = threading.Lock()
        self.ok = False
        self.node = None
        self.world_topic = None

        self.p = None              # pose do modelo no mundo
        self.r = None
        self.clamp_rel = None      # (p, R) do link da garra no modelo
        self.units = {}            # nome -> (p, R)
        self.sim_t = None
        self.prev = None           # (sim_t, p) para a velocidade medida
        self.v_meas = 0.0          # m/s da simulacao, no rumo
        self.z0 = None

        self.blocked = 0           # +1 / -1: sentido bloqueado; 0 livre
        self.blocked_at = 0.0
        self.stall_since = None
        self.block_reason = ''
        self.released_at = -1e9

        self.attached = set()
        self.attach_grip = None
        self.pubs = {}
        self.grip_limit = None
        self.candidates = []
        self.at_limit_since = None

    # ------------------------------------------------------------------ gz.transport

    def start(self):
        try:
            from forklift_sim.gz_compat import Node as GzNode, msg
            Pose_V = msg('pose_v', 'Pose_V')
        except ImportError as error:
            self.log.warning(f'sem gz.transport ({error}): sem nivelamento, bloqueio nem pega de fardos')
            return

        def wait_and_subscribe():
            self.node = GzNode()
            while True:
                topics = [t for t in self.node.topic_list()
                          if t.startswith('/world/') and t.endswith('/dynamic_pose/info')]
                if topics:
                    self.world_topic = topics[0]
                    self.node.subscribe(Pose_V, topics[0], self.on_poses)
                    self.log.info(f'corpo da empilhadeira por {topics[0]}')
                    break
                time.sleep(1.0)
            # Pose inicial de todas as unidades: o pose/info completo uma vez (o dynamic_pose pode trazer so'
            # o que se mexeu), depois desassina -- ele traz tambem todos os visuais, pesado para ficar lendo.
            full = self.world_topic.replace('/dynamic_pose/info', '/pose/info')
            self.seeded = threading.Event()
            self.node.subscribe(Pose_V, full, self.on_full_poses)
            self.seeded.wait(10.0)
            self.node.unsubscribe(full)
            with self.lock:
                names = sorted(self.units)
            self.log.info(f'{len(names)} unidades de celulose com fisica no mundo')
            self.release_all(names)
        threading.Thread(target=wait_and_subscribe, daemon=True).start()

    def on_poses(self, msg):
        model = None
        clamp = None
        units = {}
        for pose in msg.pose:
            n = pose.name
            if n == self.model:
                model = pose
            elif n == self.clamp_link:
                clamp = pose
            elif n.startswith('pulp_dyn_'):
                units[n] = pose
        if model is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

        def pr(pose):
            return (np.array([pose.position.x, pose.position.y, pose.position.z]),
                    quat_to_rot((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))
        p, r = pr(model)
        with self.lock:
            if self.prev is not None and t > self.prev[0] + 1e-4:
                d = p - self.prev[1]
                heading = r[:, 0]
                h = np.array([heading[0], heading[1], 0.0])
                n = np.linalg.norm(h)
                if n > 1e-6:
                    v = float(np.dot(d, h / n)) / (t - self.prev[0])
                    self.v_meas += (v - self.v_meas) * 0.5
            if self.prev is None or t > self.prev[0] + 1e-4:
                self.prev = (t, p)
            self.p, self.r, self.sim_t = p, r, t
            if self.z0 is None:
                self.z0 = float(p[2])
            if clamp is not None:
                self.clamp_rel = pr(clamp)
            # guarda a ultima pose de cada unidade (pode nao vir em toda mensagem)
            for n, pose in units.items():
                self.units[n] = pr(pose)
            self.ok = True

    def release_all(self, names):
        """O DetachableJoint do Gazebo instalado (10.5) NAO tem initial_attach: ao nascer, a empilhadeira fica
        presa a TODAS as unidades (12 t penduradas no braco da garra, a dezenas de metros). Ela nao virava,
        patinava, inclinava e saia do chao (23/10). Solta todas e confere no topico de estado de cada uma."""
        from forklift_sim.gz_compat import msg
        StringMsg = msg('stringmsg', 'StringMsg')
        state = {}

        def on_state(name):
            return lambda msg: state.__setitem__(name, msg.data)
        for name in names:
            self.node.subscribe(StringMsg, f'/hyster/grab/{name}/state', on_state(name))
        for attempt in range(20):
            pending = [n for n in names if state.get(n) != 'detached']
            if not pending:
                break
            for name in pending:
                self.publish(f'/hyster/grab/{name}/detach', False)
            time.sleep(0.5)
        pending = [n for n in names if state.get(n) != 'detached']
        if pending:
            self.log.error(f'unidades AINDA PRESAS na garra (estado {[(n, state.get(n)) for n in pending]}): '
                           'a empilhadeira vai arrastar 2 t por unidade')
        else:
            self.log.info(f'{len(names)} unidades soltas da garra (DetachableJoint confirmou "detached")')

    def on_full_poses(self, msg):
        if self.seeded.is_set():
            return
        with self.lock:
            for pose in msg.pose:
                if pose.name.startswith('pulp_dyn_') and pose.name not in self.units:
                    self.units[pose.name] = (
                        np.array([pose.position.x, pose.position.y, pose.position.z]),
                        quat_to_rot((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))
        self.seeded.set()

    def publish(self, topic, attach):
        from forklift_sim.gz_compat import msg
        Empty = msg('empty', 'Empty')
        pub = self.pubs.get(topic)
        if pub is None:
            pub = self.node.advertise(topic, Empty)
            self.pubs[topic] = pub
            time.sleep(0.05)   # descoberta antes da primeira mensagem
        pub.publish(Empty())

    # ------------------------------------------------------------------ base

    def tilt_and_lift(self):
        """(angulo de inclinacao, altura acima do chao) do corpo"""
        yaw = math.atan2(self.r[1, 0], self.r[0, 0])
        err = rot_to_axis_angle(rot_z(yaw) @ self.r.T)
        return float(np.linalg.norm(err[:2])), float(self.p[2] - self.z0), yaw, err

    def base_command(self, v_sim, w_sim):
        """(v_sim, w_sim) pedidos -> (linear, angular) no referencial do corpo, com nivelamento e bloqueio.
        Sem pose ainda: devolve o pedido sem mexer."""
        with self.lock:
            if not self.ok:
                return (v_sim, 0.0, 0.0), (0.0, 0.0, w_sim)
            tilt, lift, yaw, err = self.tilt_and_lift()
            now = time.monotonic()
            sign = 1 if v_sim > 1e-3 else (-1 if v_sim < -1e-3 else 0)

            # bloqueio: inverter o sentido libera; senao tenta de novo depois de RETRY
            if self.blocked and (sign == -self.blocked or now - self.blocked_at > RETRY):
                self.blocked = 0
                self.stall_since = None
                self.released_at = now
            # So' a falta de avanco bloqueia: a inclinacao nao (numa curva rapida o corpo inclina uns 2 graus e
            # isso travava a empilhadeira como uma parede invisivel -- 23/10). O nivelamento impede escalar.
            if sign and not self.blocked:
                if abs(v_sim) > 0.05 and self.v_meas * sign < STALL_RATIO * abs(v_sim):
                    # conta em tempo de SIMULACAO: Gazebo engasgado (tempo parado) nao e' obstaculo
                    if self.stall_since is None:
                        self.stall_since = self.sim_t
                    elif self.sim_t - self.stall_since > STALL_TIME:
                        self.block(sign, now, 'sem avanco (%.2f de %.2f m/s)' % (self.v_meas, v_sim))
                else:
                    self.stall_since = None
            if self.blocked and sign == self.blocked:
                v_sim = 0.0
                w_sim = 0.0

            # velocidade no mundo: horizontal no rumo + volta ao chao
            vw = np.array([v_sim * math.cos(yaw), v_sim * math.sin(yaw),
                           clampf(-K_Z * lift, -MAX_VZ, MAX_VZ)])
            ww = K_TILT * np.array([err[0], err[1], 0.0])
            n = np.linalg.norm(ww)
            if n > MAX_W_TILT:
                ww *= MAX_W_TILT / n
            ww[2] = w_sim
            rt = self.r.T
            return tuple(rt @ vw), tuple(rt @ ww)

    def block(self, sign, now, reason):
        self.blocked = sign
        self.blocked_at = now
        self.stall_since = None
        self.block_reason = reason
        self.log.info(f'base BLOQUEADA para {"frente" if sign > 0 else "re"}: {reason}')

    # ------------------------------------------------------------------ garra

    def grip_step(self, grip):
        """Recebe o fechamento pedido (m por braco) e devolve o permitido; prende/solta as unidades."""
        if not self.ok or self.clamp_rel is None or self.node is None:
            return grip
        with self.lock:
            pc_rel, rc_rel = self.clamp_rel
            pc = self.p + self.r @ pc_rel
            rc = self.r @ rc_rel
            units = dict(self.units)
        rct = rc.T
        candidates = []
        limit = None
        for name, (pu, ru) in units.items():
            r = rct @ ru
            center = rct @ (pu - pc) + r @ np.array([0.0, 0.0, UNIT_SIZE[2] / 2])
            half = 0.5 * np.abs(r) @ UNIT_SIZE
            if center[2] - half[2] > PAD_Z[1] or center[2] + half[2] < PAD_Z[0]:
                continue
            if center[1] - half[1] > PAD_Y[1] - 0.15 or center[1] + half[1] < PAD_Y[0] + 0.15:
                continue
            if center[0] - half[0] < ARM1_FACE_OPEN - 0.05 or center[0] + half[0] > ARM2_FACE_OPEN + 0.05:
                continue
            # os dois bracos encostados com a unidade centrada (o primeiro a encostar empurra)
            g = ((ARM2_FACE_OPEN - ARM1_FACE_OPEN) - 2.0 * half[0]) / 2.0
            candidates.append(name)
            limit = g if limit is None else min(limit, g)
        self.candidates = candidates
        self.grip_limit = limit

        if self.attached:
            if grip < self.attach_grip - RELEASE_OPEN:
                for name in sorted(self.attached):
                    self.publish(f'/hyster/grab/{name}/detach', False)
                self.log.info(f'garra SOLTOU {", ".join(sorted(self.attached))}')
                self.attached = set()
                self.attach_grip = None
                return grip
            return min(grip, self.attach_grip)

        if limit is None:
            self.at_limit_since = None
            return grip
        grip = min(grip, limit)
        now = time.monotonic()
        if grip < limit - ATTACH_TOL:
            self.at_limit_since = None
        elif self.at_limit_since is None:
            self.at_limit_since = now
        elif now - self.at_limit_since > ATTACH_HOLD:
            self.at_limit_since = None
            for name in candidates:
                self.publish(f'/hyster/grab/{name}/attach', True)
            self.attached = set(candidates)
            self.attach_grip = grip
            self.log.info(f'garra PEGOU {", ".join(candidates)} (fechamento {grip:.3f} m)')
        return grip

    def status(self):
        if not self.ok:
            return 'corpo: sem pose do Gazebo'
        with self.lock:
            tilt, lift, _, _ = self.tilt_and_lift()
        b = 'livre' if not self.blocked else f'BLOQUEADA {"frente" if self.blocked > 0 else "re"} ({self.block_reason})'
        g = f'segurando {len(self.attached)}' if self.attached else (
            f'{len(self.candidates)} unidade(s) entre os bracos, encosta em {self.grip_limit:.3f} m'
            if self.candidates else 'vazia')
        return (f'corpo: inclinacao {math.degrees(tilt):.1f} graus, altura {lift:+.3f} m, '
                f'v medida {self.v_meas:+.2f} | base {b} | garra {g}')
