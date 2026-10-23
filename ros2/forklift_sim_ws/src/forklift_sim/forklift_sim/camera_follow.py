"""Camera da janela do Gazebo que acompanha a empilhadeira so' por translacao.

Os modos "follow" do proprio Gazebo prendem a camera no referencial do veiculo: ela gira
junto nas curvas e nao deixa dar zoom. Aqui, a cada ciclo, a camera anda o MESMO
deslocamento que a empilhadeira andou (sem girar). Zoom, rotacao e arrasto feitos com o
mouse continuam valendo: a camera so' e' carregada junto.

Usa o gz-transport direto (sem ROS):
  /world/<mundo>/pose/info  pose da empilhadeira
  /gui/camera/pose          pose atual da camera da janela
  /gui/move_to/pose         move a camera

Uso: camera_follow [--model hyster] [--offset -8 0 3.8] [--rate 30]
"""

import argparse
import collections
import math
import re
import threading
import time

from forklift_sim.gz_compat import Node, msg

Boolean = msg('boolean', 'Boolean')
GUICamera = msg('gui_camera', 'GUICamera')
Pose = msg('pose', 'Pose')
Pose_V = msg('pose_v', 'Pose_V')


def look_at(eye, target):
    """Quaternion (w, x, y, z) de uma camera em eye olhando para target (x para frente)."""
    dx, dy, dz = (target[i] - eye[i] for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)


class CameraFollow:

    def __init__(self, model, offset):
        self.model = model
        self.offset = offset
        self.node = Node()
        self.lock = threading.Lock()
        self.vehicle = None        # (x, y, z, yaw)
        self.camera = None         # Pose
        self.carried = None        # posicao do veiculo na ultima vez que a camera foi carregada
        self.commanded = None      # ultima pose mandada (posicao, orientacao)
        self.history = collections.deque()   # (instante, posicao, orientacao) mandados no ultimo 1 s
        self.node.subscribe(Pose, '/gui/camera/pose', self.on_camera)

    def subscribe_world(self):
        while True:
            for topic in self.node.topic_list():
                if re.fullmatch(r'/world/[^/]+/pose/info', topic):
                    self.node.subscribe(Pose_V, topic, self.on_poses)
                    print(f'camera_follow: seguindo "{self.model}" em {topic}', flush=True)
                    return
            time.sleep(1.0)

    def on_camera(self, msg):
        with self.lock:
            self.camera = msg

    def on_poses(self, msg):
        for p in msg.pose:
            if p.name == self.model:
                q = p.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                with self.lock:
                    self.vehicle = (p.position.x, p.position.y, p.position.z, yaw)
                return

    def move_camera(self, position, orientation):
        req = GUICamera()
        req.pose.position.x, req.pose.position.y, req.pose.position.z = position
        req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z = orientation
        req.duration = 0.0
        self.node.request('/gui/move_to/pose', req, GUICamera, Boolean, 200)
        self.commanded = (tuple(position), tuple(orientation))
        now = time.monotonic()
        self.history.append((now, self.commanded[0], self.commanded[1]))
        while self.history and now - self.history[0][0] > 1.0:
            self.history.popleft()

    def step(self):
        with self.lock:
            vehicle, camera = self.vehicle, self.camera
        if vehicle is None or camera is None:
            return
        x, y, z, yaw = vehicle
        if self.carried is None:
            # primeira vez: atras e acima do veiculo (no sentido em que ele nasceu), olhando para ele
            ox, oy, oz = self.offset
            eye = (x + ox * math.cos(yaw) - oy * math.sin(yaw), y + ox * math.sin(yaw) + oy * math.cos(yaw), z + oz)
            self.move_camera(eye, look_at(eye, (x, y, z + 1.0)))
            self.carried = (x, y, z)
            return
        dx, dy, dz = x - self.carried[0], y - self.carried[1], z - self.carried[2]
        if abs(dx) + abs(dy) + abs(dz) < 1e-3:
            return
        # Base: a ultima pose que eu mandei. Se a pose publicada pela janela nao bate com
        # nenhuma das mandadas no ultimo segundo, foi o usuario (mouse): vale a dele.
        p, q = camera.position, camera.orientation
        published = ((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))
        base = self.commanded or published

        def close(pos, rot):
            return (sum(abs(published[0][i] - pos[i]) for i in range(3)) < 0.05 and
                    sum(abs(published[1][i] - rot[i]) for i in range(4)) < 0.01)
        if self.commanded is not None and not any(close(h[1], h[2]) for h in self.history):
            base = published
        pos, rot = base
        self.move_camera((pos[0] + dx, pos[1] + dy, pos[2] + dz), rot)
        self.carried = (x, y, z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='hyster')
    ap.add_argument('--offset', type=float, nargs=3, default=[-8.0, 0.0, 3.8])
    ap.add_argument('--rate', type=float, default=30.0)
    a, _ = ap.parse_known_args()
    follow = CameraFollow(a.model, a.offset)
    follow.subscribe_world()
    period = 1.0 / a.rate
    try:
        while True:
            follow.step()
            time.sleep(period)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
