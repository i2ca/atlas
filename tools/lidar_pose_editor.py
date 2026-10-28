#!/usr/bin/env python3
"""Editor 3D de LiDARs da empilhadeira: a Hyster (o mesmo models/hyster.obj do viewer_3D) com os
LiDARs em cima, para posicionar cada um vendo onde fica e o que a propria empilhadeira tampa.

    $ASTRO_HOME/data/empilhadeira_sim/tools/lidar_pose_editor.py [astro.ini]

MOUSE   esquerdo arrasta = gira a vista | direito/meio = arrasta | roda = zoom
        Shift+clique na empilhadeira = cola o LiDAR selecionado ali (+ "altura do suporte")
        duplo clique perto de um LiDAR = seleciona
TECLADO (com a vista em foco) W/S x  A/D y  R/F z  Q/E yaw  T/G roll  Y/H pitch  (passo do combo)
        1-5 vistas  Ctrl+Z desfaz  Ctrl+S salva  Tab proximo LiDAR
POSE    em relacao ao CARRO (base_footprint, x para a garra, z para cima) ou a sensor_board_1 (como
        no .ini). O .ini e' sempre gravado em relacao a sensor_board_1.
FEIXES  raios do LiDAR selecionado com os vertical_angles dele; "Calcular sombra" mostra em vermelho
        os que batem na empilhadeira e o % bloqueado.
AO VIVO cada mudanca vai para o param_daemon (param_tool set) e o viewer_3D move a nuvem na hora.
SALVAR  grava no .ini todos os LiDARs alterados (backup .bak-lidar_editor na 1a vez). O Gazebo
        monta os sensores pelo .ini ao subir: depois reinicie "gazebo" e "lidarN" no proccontrol_gui.
"""

import math
import os
import re
import shutil
import sys

import numpy as np
from OpenGL.GL import *   # noqa: F401,F403
from OpenGL.GLU import gluLookAt, gluPerspective, gluUnProject, gluProject
from PyQt5.QtCore import QProcess, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QKeySequence, QPainter
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout,
                             QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem, QMainWindow,
                             QMessageBox, QOpenGLWidget, QPushButton, QScrollArea, QShortcut, QSplitter,
                             QVBoxLayout, QWidget)

ASTRO_HOME = os.environ.get('ASTRO_HOME', os.path.expanduser('~/astro'))
SIM_DIR = os.path.join(ASTRO_HOME, 'data', 'empilhadeira_sim')
DEFAULT_INI = os.path.join(SIM_DIR, 'parameters_ini_new', 'astro.ini')
MODEL_OBJ = os.path.join(SIM_DIR, 'models', 'hyster.obj')
PARAM_TOOL = os.path.join(ASTRO_HOME, 'bin', 'param_tool')
POSE_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
ANGLE_KEYS = ('roll', 'pitch', 'yaw')
MODEL_KEYS = ('model', 'shot_size', 'vertical_angles', 'horizontal_angles', 'ray_order', 'time_between_shots',
              'max_range', 'min_sensing', 'max_sensing', 'range_division_factor', 'imu_frequency')
LINE_RE = r'^(%s)([ \t]+)([^#\n]*?)([ \t]*(#[^\n]*)?)$'
CLAMP_LINK = 'Base_das_garras_1'
# modelos prontos ("Modelo pronto"): os mesmos valores que estao no astro.ini da empilhadeira
PRESETS = {
    'Pandar XT-32 (ARGOS)': {
        'model': 'Pandar_XT-32', 'shot_size': '32', 'min_sensing': '1000', 'max_sensing': '24000',
        'range_division_factor': '250', 'max_range': '70.0', 'time_between_shots': '0.00002512562',
        'vertical_angles': ' '.join(str(a) for a in range(15, -17, -1)), 'horizontal_angles': '',
        'ray_order': ' '.join(str(i) for i in range(32))},
    'Hesai JT128 (hemisferico)': {
        'model': 'JT128', 'shot_size': '128', 'min_sensing': '13', 'max_sensing': '15000',
        'range_division_factor': '250', 'max_range': '60.0', 'time_between_shots': '0.000111111',
        'vertical_angles': ' '.join('%.3f' % (90.5 - i * 94.9 / 127) for i in range(128)), 'horizontal_angles': '',
        'ray_order': ' '.join(str(i) for i in range(128))},
}
COLORS = [(1.0, 0.2, 0.2), (0.1, 0.6, 1.0), (1.0, 0.6, 0.0), (0.6, 0.2, 1.0), (0.0, 0.75, 0.4), (1.0, 0.2, 0.8),
          (0.5, 0.5, 0.0), (0.0, 0.6, 0.6)]


# ============================================================================ geometria
def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rpy_of(r):
    return (math.atan2(r[2, 1], r[2, 2]), -math.asin(max(-1.0, min(1.0, r[2, 0]))), math.atan2(r[1, 0], r[0, 0]))


def make_tf(pose):
    t = np.eye(4)
    t[:3, :3] = rotation(*pose[3:])
    t[:3, 3] = pose[:3]
    return t


def tf_pose(t):
    return list(t[:3, 3]) + list(rpy_of(t[:3, :3]))


def ray_mesh(origin, dirs, v0, e1, e2, t_min=0.03, batch=96):
    """Distancia do primeiro triangulo em cada raio (inf se nao bate). Moller-Trumbore vetorizado."""
    out = np.full(len(dirs), np.inf)
    tvec = origin[None, :] - v0
    q = np.cross(tvec, e1)
    e2q = np.einsum('ij,ij->i', e2, q)
    for s in range(0, len(dirs), batch):
        d = dirs[s:s + batch]
        p = np.cross(d[:, None, :], e2[None, :, :])
        det = np.einsum('tk,rtk->rt', e1, p)
        ok = np.abs(det) > 1e-9
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        u = np.einsum('tk,rtk->rt', tvec, p) * inv
        v = np.einsum('rk,tk->rt', d, q) * inv
        t = e2q[None, :] * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > t_min)
        t = np.where(hit, t, np.inf)
        out[s:s + batch] = t.min(axis=1)
    return out


def load_obj(path):
    """hyster.obj (eixos do viewer_3D: x frente, y cima, z direita) -> triangulos no carro (z cima)."""
    verts, norms, mats, colors = [], [], {}, {}
    mtl_name, cur = None, 'default'
    faces = []
    for line in open(path):
        p = line.split()
        if not p:
            continue
        if p[0] == 'v':
            x, y, z = map(float, p[1:4])
            verts.append((x, -z, y))
        elif p[0] == 'vn':
            x, y, z = map(float, p[1:4])
            norms.append((x, -z, y))
        elif p[0] == 'mtllib':
            mtl_name = os.path.join(os.path.dirname(path), p[1])
        elif p[0] == 'usemtl':
            cur = p[1]
        elif p[0] == 'f':
            idx = [int(c.split('/')[0]) - 1 for c in p[1:4]]
            nidx = [int(c.split('/')[2]) - 1 if c.count('/') == 2 and c.split('/')[2] else None for c in p[1:4]]
            faces.append((cur, idx, nidx))
    if mtl_name and os.path.exists(mtl_name):
        name = None
        for line in open(mtl_name):
            p = line.split()
            if p and p[0] == 'newmtl':
                name = p[1]
            elif p and p[0] == 'Kd' and name:
                colors[name] = tuple(map(float, p[1:4]))
    verts = np.array(verts, np.float32)
    norms = np.array(norms, np.float32) if norms else None
    tris = np.array([f[1] for f in faces], np.int64)
    for m in sorted({f[0] for f in faces}):
        sel = np.array([f[0] == m for f in faces])
        tv = verts[tris[sel]].reshape(-1, 3)
        if norms is not None and all(n is not None for f in faces for n in f[2]):
            nid = np.array([f[2] for f in faces], np.int64)[sel]
            tn = norms[nid].reshape(-1, 3)
        else:
            a, b, c = verts[tris[sel][:, 0]], verts[tris[sel][:, 1]], verts[tris[sel][:, 2]]
            n = np.cross(b - a, c - a)
            n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
            tn = np.repeat(n, 3, axis=0)
        mats[m] = (np.ascontiguousarray(tv, np.float32), np.ascontiguousarray(tn, np.float32),
                   colors.get(m, (0.7, 0.7, 0.7)))
    all_t = verts[tris].astype(np.float64)
    return mats, all_t


# ============================================================================ .ini
def read_ini_values(path):
    values = {}
    for line in open(path, errors='replace'):
        m = re.match(r'^([A-Za-z_][A-Za-z0-9_]*)[ \t]+([^#\n]*)', line)
        if m and m.group(1) not in values:
            values[m.group(1)] = m.group(2).strip()
    return values


def num(values, name, default=0.0):
    try:
        return float(values.get(name, default).split()[0])
    except (ValueError, IndexError, AttributeError):
        return float(default)


# ============================================================================ vista 3D
class View3D(QOpenGLWidget):
    def __init__(self, editor):
        super().__init__()
        self.ed = editor
        self.setFocusPolicy(Qt.StrongFocus)
        self.target = np.array([-0.3, 0.0, 1.5])
        self.dist, self.yaw, self.pitch = 9.0, math.radians(-140), math.radians(25)
        self.last = None
        self.mv = self.pr = self.vp = None
        self.model_lists = None     # display lists da empilhadeira (uma por material)

    def set_view(self, yaw_deg, pitch_deg, dist=None):
        self.yaw, self.pitch = math.radians(yaw_deg), math.radians(pitch_deg)
        if dist:
            self.dist = dist
        self.update()

    def initializeGL(self):
        glClearColor(0.16, 0.17, 0.19, 1)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_NORMALIZE)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.85, 0.85, 0.85, 1))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.35, 0.35, 0.35, 1))

    def eye(self):
        cp = math.cos(self.pitch)
        return self.target + self.dist * np.array([cp * math.cos(self.yaw), cp * math.sin(self.yaw), math.sin(self.pitch)])

    def paintGL(self):
        painter = QPainter(self)
        painter.beginNativePainting()
        dpr = self.devicePixelRatioF()
        w, h = int(self.width() * dpr), int(self.height() * dpr)
        glViewport(0, 0, w, h)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, w / max(h, 1), 0.05, 300.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        e = self.eye()
        gluLookAt(e[0], e[1], e[2], *self.target, 0, 0, 1)
        glLightfv(GL_LIGHT0, GL_POSITION, (e[0], e[1], e[2] + 3, 1))
        self.mv = glGetDoublev(GL_MODELVIEW_MATRIX)
        self.pr = glGetDoublev(GL_PROJECTION_MATRIX)
        self.vp = glGetIntegerv(GL_VIEWPORT)

        self.draw_grid()
        self.draw_axes(np.eye(4), 1.0, 3.0)
        ed = self.ed
        if ed.chk_board.isChecked():
            self.draw_axes(ed.board_tf, 0.5, 2.0)
        self.draw_markers()
        self.draw_lidars()
        self.draw_rays()
        self.draw_model()
        painter.endNativePainting()
        self.draw_labels(painter)
        painter.end()

    def draw_grid(self):
        glDisable(GL_LIGHTING)
        glLineWidth(1)
        glBegin(GL_LINES)
        for i in range(-8, 9):
            c = 0.32 if i else 0.45
            glColor3f(c, c, c)
            glVertex3f(i, -8, 0); glVertex3f(i, 8, 0)
            glVertex3f(-8, i, 0); glVertex3f(8, i, 0)
        glEnd()

    def draw_axes(self, tf, size, width):
        glDisable(GL_LIGHTING)
        glLineWidth(width)
        o = tf[:3, 3]
        glBegin(GL_LINES)
        for k, c in enumerate(((1, 0.2, 0.2), (0.2, 1, 0.2), (0.3, 0.5, 1))):
            glColor3f(*c)
            glVertex3f(*o)
            glVertex3f(*(o + tf[:3, k] * size))
        glEnd()
        glLineWidth(1)

    def draw_model(self):
        ed = self.ed
        if not ed.chk_model.isChecked() or not ed.mesh:
            return
        alpha = ed.alpha
        glEnable(GL_LIGHTING)
        if alpha < 0.99:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glDepthMask(GL_FALSE)
        # display list (montada uma vez): glVertexPointer do PyOpenGL nao acha o contexto do Qt
        # em EGL/Wayland ("Attempt to retrieve context when no valid context")
        if self.model_lists is None:
            self.model_lists = []
            for _m, (tv, tn, col) in ed.mesh.items():
                lst = glGenLists(1)
                glNewList(lst, GL_COMPILE)
                glBegin(GL_TRIANGLES)
                for v, n in zip(tv.tolist(), tn.tolist()):
                    glNormal3f(*n)
                    glVertex3f(*v)
                glEnd()
                glEndList()
                self.model_lists.append((lst, col))
        for lst, col in self.model_lists:
            glColor4f(col[0], col[1], col[2], alpha)
            glCallList(lst)
        glDepthMask(GL_TRUE)
        glDisable(GL_BLEND)
        glDisable(GL_LIGHTING)

    def box(self, tf, sx, sy, sz, color, lit=True):
        corners = np.array([[x, y, z, 1] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)]) @ tf.T
        faces = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
        if lit:
            glEnable(GL_LIGHTING)
        glColor3f(*color)
        glBegin(GL_QUADS)
        for f in faces:
            a, b, c = corners[f[0], :3], corners[f[1], :3], corners[f[2], :3]
            n = np.cross(b - a, c - a)
            glNormal3f(*(n / (np.linalg.norm(n) + 1e-9)))
            for i in f:
                glVertex3f(*corners[i, :3])
        glEnd()
        glDisable(GL_LIGHTING)

    def draw_lidars(self):
        ed = self.ed
        for n in ed.lidar_ids:
            if not ed.visible.get(n, False):
                continue
            tf = ed.lidar_tf(n)
            col = COLORS[n % len(COLORS)]
            sel = (n == ed.cur)
            self.box(tf, 0.06, 0.06, 0.05, col)
            self.draw_axes(tf, 0.35 if sel else 0.2, 3.0 if sel else 1.5)
            if sel:
                glDisable(GL_DEPTH_TEST)
                glLineWidth(2)
                glColor3f(1, 1, 0)
                glBegin(GL_LINE_LOOP)
                for a in range(24):
                    th = 2 * math.pi * a / 24
                    p = tf @ np.array([0.13 * math.cos(th), 0.13 * math.sin(th), 0, 1])
                    glVertex3f(*p[:3])
                glEnd()
                glEnable(GL_DEPTH_TEST)
                glLineWidth(1)

    def draw_markers(self):
        for name, tf, col in self.ed.markers:
            if not self.ed.chk_markers.isChecked():
                return
            self.box(tf, 0.04, 0.04, 0.04, col)
            self.draw_axes(tf, 0.18, 1.5)

    def draw_rays(self):
        ed = self.ed
        if not ed.chk_rays.isChecked() or ed.cur is None:
            return
        origin, dirs = ed.ray_dirs(ed.cur)
        if dirs is None:
            return
        L = ed.spin_ray_len.value()
        hits = ed.shadow if (ed.shadow is not None and len(ed.shadow) == len(dirs)) else None
        glDisable(GL_LIGHTING)
        glLineWidth(1)
        glBegin(GL_LINES)
        for i, d in enumerate(dirs):
            t = hits[i] if hits is not None else np.inf
            if t < L:
                glColor4f(1.0, 0.1, 0.1, 1)
                end = origin + d * t
            else:
                glColor4f(0.25, 0.55, 0.35, 1)
                end = origin + d * L
            glVertex3f(*origin)
            glVertex3f(*end)
        glEnd()

    def project(self, p):
        if self.mv is None:
            return None
        try:
            x, y, z = gluProject(p[0], p[1], p[2], self.mv, self.pr, self.vp)
        except Exception:
            return None
        if z > 1.0:
            return None
        dpr = self.devicePixelRatioF()
        return x / dpr, (self.vp[3] - y) / dpr

    def draw_labels(self, painter):
        ed = self.ed
        painter.setFont(QFont('Sans', 9, QFont.Bold))
        for n in ed.lidar_ids:
            if not ed.visible.get(n, False):
                continue
            s = self.project(ed.lidar_tf(n)[:3, 3] + np.array([0, 0, 0.15]))
            if s:
                c = COLORS[n % len(COLORS)]
                painter.setPen(QColor(255, 255, 0) if n == ed.cur else QColor(int(c[0] * 255), int(c[1] * 255), int(c[2] * 255)))
                painter.drawText(int(s[0]) - 20, int(s[1]), 'lidar%d' % n)
        if ed.chk_markers.isChecked():
            painter.setFont(QFont('Sans', 8))
            painter.setPen(QColor(200, 200, 200))
            for name, tf, _c in ed.markers:
                s = self.project(tf[:3, 3] + np.array([0, 0, 0.1]))
                if s:
                    painter.drawText(int(s[0]) - 15, int(s[1]), name)
        painter.setPen(QColor(220, 220, 220))
        painter.setFont(QFont('Sans', 9))
        painter.drawText(8, self.height() - 8, ed.view_hint)

    # ---------------------------------------------------------------- mouse
    def mouse_ray(self, ev):
        if self.mv is None:
            return None, None
        dpr = self.devicePixelRatioF()
        x, y = ev.x() * dpr, self.vp[3] - ev.y() * dpr
        a = np.array(gluUnProject(x, y, 0.0, self.mv, self.pr, self.vp))
        b = np.array(gluUnProject(x, y, 1.0, self.mv, self.pr, self.vp))
        d = b - a
        return a, d / np.linalg.norm(d)

    def mousePressEvent(self, ev):
        self.last = ev.pos()
        if ev.button() == Qt.LeftButton and ev.modifiers() & Qt.ShiftModifier:
            o, d = self.mouse_ray(ev)
            if o is not None:
                self.ed.place_on_model(o, d)
            self.last = None

    def mouseDoubleClickEvent(self, ev):
        best, bd = None, 30.0
        for n in self.ed.lidar_ids:
            if not self.ed.visible.get(n, False):
                continue
            s = self.project(self.ed.lidar_tf(n)[:3, 3])
            if s:
                d = math.hypot(s[0] - ev.x(), s[1] - ev.y())
                if d < bd:
                    best, bd = n, d
        if best is not None:
            self.ed.select(best)

    def mouseMoveEvent(self, ev):
        if self.last is None:
            return
        dx, dy = ev.x() - self.last.x(), ev.y() - self.last.y()
        self.last = ev.pos()
        if ev.buttons() & Qt.LeftButton:
            self.yaw -= dx * 0.008
            self.pitch = max(-1.5, min(1.5, self.pitch + dy * 0.008))
        elif ev.buttons() & (Qt.RightButton | Qt.MiddleButton):
            e = self.eye()
            fwd = self.target - e
            fwd /= np.linalg.norm(fwd)
            right = np.cross(fwd, [0, 0, 1])
            right /= np.linalg.norm(right) + 1e-9
            up = np.cross(right, fwd)
            k = self.dist * 0.0015
            self.target += (-dx * right + dy * up) * k
        self.update()

    def mouseReleaseEvent(self, ev):
        self.last = None

    def wheelEvent(self, ev):
        self.dist = max(0.5, min(80.0, self.dist * (0.88 if ev.angleDelta().y() > 0 else 1.0 / 0.88)))
        self.update()

    def keyPressEvent(self, ev):
        if not self.ed.key_nudge(ev):
            super().keyPressEvent(ev)


# ============================================================================ janela
class Editor(QMainWindow):
    def __init__(self, ini_path):
        super().__init__()
        self.ini_path = ini_path
        self.setWindowTitle('Editor de LiDARs - empilhadeira')
        self.values = read_ini_values(ini_path)
        self.lidar_ids = sorted(int(m.group(1)) for k in self.values for m in [re.match(r'^lidar(\d+)_model$', k)] if m)
        self.poses = {}             # n -> pose em relacao a sensor_board_1 [x y z roll pitch yaw]
        self.models = {}            # n -> {chave: texto}
        self.links = {}             # n -> link onde o LiDAR esta' preso no Gazebo ('' = base_footprint)
        self.dirty = set()
        self.undo = []
        self.visible = {n: n <= 4 for n in self.lidar_ids}
        self.cur = None
        self.shadow = None
        self.clip = None
        self.loading = False
        self.backup_done = False
        self.view_hint = ''
        self.alpha = 1.0
        self.pending = {}
        self.live_fail = 0
        try:
            self.mesh, self.tris = load_obj(MODEL_OBJ)
        except (OSError, ValueError) as e:
            self.mesh, self.tris = {}, None
            print('sem modelo 3D:', e)
        if self.tris is not None:
            self.v0 = self.tris[:, 0]
            self.e1 = self.tris[:, 1] - self.v0
            self.e2 = self.tris[:, 2] - self.v0
        self.load_from_values()
        self.build_ui()
        self.live_timer = QTimer(self)
        self.live_timer.setSingleShot(True)
        self.live_timer.timeout.connect(self.flush_live)
        self.shadow_timer = QTimer(self)
        self.shadow_timer.setSingleShot(True)
        self.shadow_timer.timeout.connect(self.compute_shadow)
        if self.lidar_ids:
            self.select(1 if 1 in self.lidar_ids else self.lidar_ids[0])

    # ---------------------------------------------------------------- dados
    def load_from_values(self):
        v = self.values
        self.board = [num(v, 'sensor_board_1_' + k) for k in POSE_KEYS]
        self.board_tf = make_tf(self.board)
        for n in self.lidar_ids:
            self.poses[n] = [num(v, 'lidar%d_%s' % (n, k)) for k in POSE_KEYS]
            self.models[n] = {k: v.get('lidar%d_%s' % (n, k), '') for k in MODEL_KEYS}
            self.links[n] = (v.get('lidar%d_link' % n) or '').split()[0] if v.get('lidar%d_link' % n) else ''
        self.markers = []
        for name, prefix, col in (('xsens', 'xsens_', (0.9, 0.9, 0.9)), ('gps', 'gps_nmea_1_', (1.0, 1.0, 0.3))):
            if prefix + 'x' in v:
                p = [num(v, prefix + k) for k in POSE_KEYS]
                self.markers.append((name, self.board_tf @ make_tf(p), col))
        for c in range(1, 4):
            pre = 'intelbras%d_' % c
            if pre + 'x' in v:
                p = [num(v, pre + k) for k in POSE_KEYS]
                self.markers.append(('cam%d' % c, self.board_tf @ make_tf(p), (0.3, 0.9, 1.0)))

    def lidar_tf(self, n):
        return self.board_tf @ make_tf(self.poses[n])

    def car_pose(self, n):
        return tf_pose(self.lidar_tf(n))

    def ray_dirs(self, n):
        tf = self.lidar_tf(n)
        try:
            vang = [float(a) for a in self.models[n].get('vertical_angles', '').split()]
        except ValueError:
            vang = []
        if not vang:
            vang = [-15, -10, -5, 0, 5, 10, 15]
        vang = sorted(set(vang))
        maxch = int(self.spin_rays_ch.value())
        if len(vang) > maxch:
            idx = np.linspace(0, len(vang) - 1, maxch).round().astype(int)
            vang = [vang[i] for i in sorted(set(idx))]
        step = self.spin_rays_az.value()
        az = np.radians(np.arange(0.0, 360.0, step))
        el = np.radians(np.array(vang))
        A, E = np.meshgrid(az, el)
        d = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], axis=-1).reshape(-1, 3)
        d = d @ tf[:3, :3].T
        return tf[:3, 3], d

    # ---------------------------------------------------------------- interface
    def build_ui(self):
        self.view = View3D(self)
        side = QWidget()
        lay = QVBoxLayout(side)

        # lista de LiDARs
        box_l = QGroupBox('LiDARs (marque para mostrar; clique para editar)')
        bl = QVBoxLayout(box_l)
        self.list = QListWidget()
        for n in self.lidar_ids:
            it = QListWidgetItem(self.label(n))
            it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
            it.setCheckState(Qt.Checked if self.visible[n] else Qt.Unchecked)
            it.setData(Qt.UserRole, n)
            c = COLORS[n % len(COLORS)]
            it.setForeground(QColor(int(c[0] * 255), int(c[1] * 255), int(c[2] * 255)))
            self.list.addItem(it)
        self.list.setMaximumHeight(150)
        self.list.itemChanged.connect(self.list_changed)
        self.list.currentItemChanged.connect(lambda it, _p: it and self.select(it.data(Qt.UserRole)))
        bl.addWidget(self.list)
        lay.addWidget(box_l)

        # pose
        box_p = QGroupBox('Pose')
        gp = QGridLayout(box_p)
        self.frame = QComboBox()
        self.frame.addItems(['no CARRO (base_footprint)', 'na sensor_board_1 (como no .ini)'])
        self.frame.currentIndexChanged.connect(self.refresh_fields)
        gp.addWidget(QLabel('referencial:'), 0, 0)
        gp.addWidget(self.frame, 0, 1, 1, 2)
        self.spins = {}
        for i, k in enumerate(POSE_KEYS):
            s = QDoubleSpinBox()
            s.setKeyboardTracking(False)
            if k in ANGLE_KEYS:
                s.setRange(-360, 360); s.setDecimals(2); s.setSuffix(' graus')
            else:
                s.setRange(-20, 20); s.setDecimals(3); s.setSuffix(' m')
            s.valueChanged.connect(self.fields_changed)
            gp.addWidget(QLabel(k), 1 + (i // 3) * 2, i % 3)
            gp.addWidget(s, 2 + (i // 3) * 2, i % 3)
            self.spins[k] = s
        self.step = QComboBox()
        for label, lin, ang in (('1 cm / 0,5 grau', 0.01, 0.5), ('5 cm / 2 graus', 0.05, 2.0),
                                ('10 cm / 5 graus', 0.10, 5.0), ('25 cm / 15 graus', 0.25, 15.0)):
            self.step.addItem(label, (lin, ang))
        self.step.setCurrentIndex(1)
        self.step.currentIndexChanged.connect(self.step_changed)
        gp.addWidget(QLabel('passo:'), 5, 0)
        gp.addWidget(self.step, 5, 1, 1, 2)
        self.spin_mount = QDoubleSpinBox()
        self.spin_mount.setRange(0.0, 1.0); self.spin_mount.setDecimals(2); self.spin_mount.setSingleStep(0.01)
        self.spin_mount.setValue(0.06); self.spin_mount.setSuffix(' m')
        gp.addWidget(QLabel('altura do suporte\n(Shift+clique):'), 6, 0)
        gp.addWidget(self.spin_mount, 6, 1)
        self.chk_level = QCheckBox('nivelar ao colar')
        self.chk_level.setChecked(True)
        gp.addWidget(self.chk_level, 6, 2)
        self.chk_clamp = QCheckBox('preso na garra (%s): no Gazebo sobe/desce/desliza junto' % CLAMP_LINK)
        self.chk_clamp.toggled.connect(self.clamp_toggled)
        gp.addWidget(self.chk_clamp, 10, 0, 1, 3)
        row = QHBoxLayout()
        for text, fn in (('Desfazer', self.do_undo), ('Copiar', self.copy_pose), ('Colar', self.paste_pose),
                         ('Voltar ao .ini', self.revert)):
            b = QPushButton(text); b.clicked.connect(fn); row.addWidget(b)
        gp.addLayout(row, 7, 0, 1, 3)
        row2 = QHBoxLayout()
        self.mirror_src = QComboBox()
        for n in self.lidar_ids:
            self.mirror_src.addItem('lidar%d' % n, n)
        b = QPushButton('Espelhar (esq<->dir) de:')
        b.clicked.connect(self.mirror)
        row2.addWidget(b); row2.addWidget(self.mirror_src)
        gp.addLayout(row2, 8, 0, 1, 3)
        self.info = QLabel()
        self.info.setWordWrap(True)
        self.info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        gp.addWidget(self.info, 9, 0, 1, 3)
        lay.addWidget(box_p)

        # modelo
        box_m = QGroupBox('Modelo (viewer_3D/forklift_lidar_sim leem ao subir)')
        fm = QFormLayout(box_m)
        rowc = QHBoxLayout()
        self.copy_src = QComboBox()
        for n in self.lidar_ids:
            self.copy_src.addItem('lidar%d (%s)' % (n, (self.models[n].get('model') or '?').split()[0]), n)
        bcp = QPushButton('Copiar modelo de')
        bcp.clicked.connect(self.copy_model)
        rowc.addWidget(bcp); rowc.addWidget(self.copy_src)
        fm.addRow(rowc)
        rowp = QHBoxLayout()
        self.preset = QComboBox()
        self.preset.addItems(list(PRESETS))
        bpre = QPushButton('Usar modelo pronto')
        bpre.clicked.connect(self.apply_preset)
        rowp.addWidget(bpre); rowp.addWidget(self.preset)
        fm.addRow(rowp)
        self.edits = {}
        for k in MODEL_KEYS:
            e = QLineEdit()
            e.editingFinished.connect(lambda key=k: self.model_edited(key))
            fm.addRow(k, e)
            self.edits[k] = e
        lay.addWidget(box_m)

        # visualizacao
        box_v = QGroupBox('Ver')
        gv = QGridLayout(box_v)
        self.chk_model = QCheckBox('empilhadeira'); self.chk_model.setChecked(True)
        self.chk_markers = QCheckBox('cameras/xsens/GPS'); self.chk_markers.setChecked(True)
        self.chk_board = QCheckBox('sensor_board_1'); self.chk_board.setChecked(True)
        self.chk_rays = QCheckBox('feixes do selecionado'); self.chk_rays.setChecked(True)
        self.chk_auto = QCheckBox('sombra automatica'); self.chk_auto.setChecked(True)
        for i, c in enumerate((self.chk_model, self.chk_markers, self.chk_board, self.chk_rays, self.chk_auto)):
            c.toggled.connect(lambda _x: self.view.update())
            gv.addWidget(c, i // 3, i % 3)
        self.chk_auto.toggled.connect(lambda on: on and self.schedule_shadow())
        self.combo_alpha = QComboBox()
        self.combo_alpha.addItems(['opaca', 'meio transparente', 'bem transparente'])
        self.combo_alpha.currentIndexChanged.connect(self.alpha_changed)
        gv.addWidget(QLabel('empilhadeira:'), 2, 0); gv.addWidget(self.combo_alpha, 2, 1, 1, 2)
        self.spin_ray_len = QDoubleSpinBox(); self.spin_ray_len.setRange(0.5, 30); self.spin_ray_len.setValue(2.5)
        self.spin_ray_len.setSuffix(' m'); self.spin_ray_len.valueChanged.connect(lambda _v: self.view.update())
        self.spin_rays_az = QDoubleSpinBox(); self.spin_rays_az.setRange(1, 30); self.spin_rays_az.setValue(10)
        self.spin_rays_az.setSuffix(' graus'); self.spin_rays_az.valueChanged.connect(lambda _v: self.schedule_shadow())
        self.spin_rays_ch = QDoubleSpinBox(); self.spin_rays_ch.setRange(2, 128); self.spin_rays_ch.setDecimals(0)
        self.spin_rays_ch.setValue(8); self.spin_rays_ch.valueChanged.connect(lambda _v: self.schedule_shadow())
        gv.addWidget(QLabel('comprimento:'), 3, 0); gv.addWidget(self.spin_ray_len, 3, 1)
        gv.addWidget(QLabel('azimute a cada:'), 4, 0); gv.addWidget(self.spin_rays_az, 4, 1)
        gv.addWidget(QLabel('canais (no max):'), 5, 0); gv.addWidget(self.spin_rays_ch, 5, 1)
        bsh = QPushButton('Calcular sombra'); bsh.clicked.connect(self.compute_shadow)
        gv.addWidget(bsh, 3, 2, 3, 1)
        rowv = QHBoxLayout()
        for text, args in (('1 Frente', (180, 10, 9)), ('2 Lado', (90, 10, 9)), ('3 Cima', (180, 89, 11)),
                           ('4 Tras', (0, 10, 9)), ('5 3D', (-140, 25, 9))):
            b = QPushButton(text); b.clicked.connect(lambda _x, a=args: self.view.set_view(*a)); rowv.addWidget(b)
        bf = QPushButton('No LiDAR'); bf.clicked.connect(self.focus_lidar); rowv.addWidget(bf)
        gv.addLayout(rowv, 6, 0, 1, 3)
        self.shadow_label = QLabel()
        gv.addWidget(self.shadow_label, 7, 0, 1, 3)
        lay.addWidget(box_v)

        # salvar / ao vivo
        box_s = QGroupBox('Salvar')
        vs = QVBoxLayout(box_s)
        self.chk_live = QCheckBox('mandar ao vivo para o param_daemon (o viewer_3D move a nuvem na hora)')
        self.chk_live.setChecked(True)
        vs.addWidget(self.chk_live)
        bsave = QPushButton('Salvar no .ini (todos os alterados)')
        bsave.setMinimumHeight(32)
        bsave.clicked.connect(self.save_ini)
        vs.addWidget(bsave)
        vs.addWidget(QLabel('Arquivo: ' + self.ini_path))
        self.status = QLabel()
        self.status.setWordWrap(True)
        vs.addWidget(self.status)
        lay.addWidget(box_s)
        lay.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidget(side)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(380)
        split = QSplitter()
        split.addWidget(self.view)
        split.addWidget(scroll)
        split.setStretchFactor(0, 3)
        split.setSizes([1050, 450])
        self.setCentralWidget(split)
        QShortcut(QKeySequence('Ctrl+Z'), self, self.do_undo)
        QShortcut(QKeySequence('Ctrl+S'), self, self.save_ini)
        self.step_changed()
        self.view_hint = ('esq. gira | dir. arrasta | roda zoom | Shift+clique cola | duplo clique seleciona | '
                          'W/S A/D R/F Q/E T/G Y/H')

    # ---------------------------------------------------------------- selecao e campos
    def select(self, n):
        if n not in self.poses:
            return
        self.cur = n
        for i in range(self.list.count()):
            if self.list.item(i).data(Qt.UserRole) == n and self.list.currentRow() != i:
                self.list.blockSignals(True); self.list.setCurrentRow(i); self.list.blockSignals(False)
        if not self.visible.get(n):
            self.set_visible(n, True)
        self.loading = True
        for k in MODEL_KEYS:
            self.edits[k].setText(self.models[n].get(k, ''))
        self.chk_clamp.setChecked(bool(self.links.get(n)))
        self.loading = False
        self.refresh_fields()
        self.shadow = None
        self.schedule_shadow()

    def set_visible(self, n, on):
        self.visible[n] = on
        for i in range(self.list.count()):
            it = self.list.item(i)
            if it.data(Qt.UserRole) == n:
                self.list.blockSignals(True)
                it.setCheckState(Qt.Checked if on else Qt.Unchecked)
                self.list.blockSignals(False)
        self.view.update()

    def list_changed(self, it):
        self.visible[it.data(Qt.UserRole)] = (it.checkState() == Qt.Checked)
        self.view.update()

    def shown_pose(self):
        return self.car_pose(self.cur) if self.frame.currentIndex() == 0 else list(self.poses[self.cur])

    def refresh_fields(self):
        if self.cur is None:
            return
        p = self.shown_pose()
        self.loading = True
        for k, v in zip(POSE_KEYS, p):
            self.spins[k].setValue(math.degrees(v) if k in ANGLE_KEYS else v)
        self.loading = False
        self.update_info()
        self.view.update()

    def update_info(self):
        n = self.cur
        c, b = self.car_pose(n), self.poses[n]
        fmt = lambda p: 'x %.3f y %.3f z %.3f | roll %.1f pitch %.1f yaw %.1f graus' % (
            p[0], p[1], p[2], math.degrees(p[3]), math.degrees(p[4]), math.degrees(p[5]))
        self.info.setText('<b>lidar%d</b>%s%s<br>carro: %s<br>sensor_board_1: %s<br>altura do chao: %.2f m' % (
            n, ' <font color="#e0a000">(alterado)</font>' if n in self.dirty else '',
            ' &mdash; preso na garra (pose com a garra em repouso)' if self.links.get(n) else '', fmt(c), fmt(b), c[2]))

    def step_changed(self):
        lin, ang = self.step.currentData()
        for k, s in self.spins.items():
            s.setSingleStep(ang if k in ANGLE_KEYS else lin)

    def fields_changed(self):
        if self.loading or self.cur is None:
            return
        p = [math.radians(self.spins[k].value()) if k in ANGLE_KEYS else self.spins[k].value() for k in POSE_KEYS]
        if self.frame.currentIndex() == 0:
            p = tf_pose(np.linalg.inv(self.board_tf) @ make_tf(p))
        self.set_pose(self.cur, p, refresh=False)

    def set_pose(self, n, p, refresh=True, undo=True):
        if undo:
            self.undo.append((n, list(self.poses[n])))
            del self.undo[:-200]
        self.poses[n] = [float(x) for x in p]
        self.dirty.add(n)
        for k, v in zip(POSE_KEYS, self.poses[n]):
            self.pending['lidar%d_%s' % (n, k)] = '%.6f' % v
        if self.chk_live.isChecked():
            self.live_timer.start(150)
        if refresh:
            self.refresh_fields()
        else:
            self.update_info()
            self.view.update()
        self.shadow = None
        self.schedule_shadow()

    # ---------------------------------------------------------------- acoes
    def key_nudge(self, ev):
        if self.cur is None:
            return False
        k = ev.key()
        if k == Qt.Key_Tab:
            vis = [n for n in self.lidar_ids if self.visible.get(n)] or self.lidar_ids
            i = vis.index(self.cur) if self.cur in vis else -1
            self.select(vis[(i + 1) % len(vis)])
            return True
        if Qt.Key_1 <= k <= Qt.Key_5:
            self.view.set_view(*((180, 10, 9), (90, 10, 9), (180, 89, 11), (0, 10, 9), (-140, 25, 9))[k - Qt.Key_1])
            return True
        lin, ang = self.step.currentData()
        moves = {Qt.Key_W: (0, lin), Qt.Key_S: (0, -lin), Qt.Key_A: (1, lin), Qt.Key_D: (1, -lin),
                 Qt.Key_R: (2, lin), Qt.Key_F: (2, -lin), Qt.Key_T: (3, ang), Qt.Key_G: (3, -ang),
                 Qt.Key_Y: (4, ang), Qt.Key_H: (4, -ang), Qt.Key_Q: (5, ang), Qt.Key_E: (5, -ang)}
        if k not in moves:
            return False
        i, d = moves[k]
        car = self.car_pose(self.cur)
        car[i] += math.radians(d) if i >= 3 else d
        self.set_pose(self.cur, tf_pose(np.linalg.inv(self.board_tf) @ make_tf(car)))
        return True

    def place_on_model(self, o, d):
        if self.cur is None or self.tris is None:
            return
        t = ray_mesh(o, d[None, :], self.v0, self.e1, self.e2, t_min=0.0)[0]
        if not np.isfinite(t):
            self.status.setText('Shift+clique fora da empilhadeira')
            return
        hit = o + d * t
        # normal do triangulo atingido, virada para a camera
        idx = self.nearest_triangle(hit)
        n = np.cross(self.e1[idx], self.e2[idx])
        n /= np.linalg.norm(n) + 1e-12
        if np.dot(n, d) > 0:
            n = -n
        pos = hit + n * self.spin_mount.value()
        car = self.car_pose(self.cur)
        car[:3] = list(pos)
        if self.chk_level.isChecked():
            car[3] = car[4] = 0.0
        self.set_pose(self.cur, tf_pose(np.linalg.inv(self.board_tf) @ make_tf(car)))
        self.status.setText('lidar%d colado em %.2f %.2f %.2f (carro)' % (self.cur, *pos))

    def nearest_triangle(self, p):
        c = self.v0 + (self.e1 + self.e2) / 3.0
        return int(np.argmin(np.sum((c - p) ** 2, axis=1)))

    def do_undo(self):
        if not self.undo:
            self.status.setText('nada para desfazer')
            return
        n, p = self.undo.pop()
        self.select(n)
        self.set_pose(n, p, undo=False)

    def copy_pose(self):
        self.clip = self.car_pose(self.cur)
        self.status.setText('pose do lidar%d copiada (no carro)' % self.cur)

    def paste_pose(self):
        if self.clip is None:
            return
        self.set_pose(self.cur, tf_pose(np.linalg.inv(self.board_tf) @ make_tf(self.clip)))

    def mirror(self):
        src = self.mirror_src.currentData()
        p = self.car_pose(src)
        p = [p[0], -p[1], p[2], -p[3], p[4], -p[5]]
        self.set_pose(self.cur, tf_pose(np.linalg.inv(self.board_tf) @ make_tf(p)))
        self.status.setText('lidar%d = espelho de lidar%d (y, roll e yaw invertidos)' % (self.cur, src))

    def revert(self):
        n = self.cur
        self.undo.append((n, list(self.poses[n])))
        orig = read_ini_values(self.ini_path)
        self.set_pose(n, [num(orig, 'lidar%d_%s' % (n, k)) for k in POSE_KEYS], undo=False)
        self.models[n] = {k: orig.get('lidar%d_%s' % (n, k), '') for k in MODEL_KEYS}
        self.select(n)

    def clamp_toggled(self, on):
        if self.loading or self.cur is None:
            return
        self.links[self.cur] = CLAMP_LINK if on else ''
        self.dirty.add(self.cur)
        self.relabel(self.cur)
        self.update_info()
        self.view.update()
        self.status.setText('lidar%d %s -- vale no Gazebo depois de salvar e reiniciar o "gazebo"' % (
            self.cur, 'preso na garra' if on else 'fixo no carro'))

    def apply_preset(self):
        name = self.preset.currentText()
        vals = PRESETS[name]
        n = self.cur
        self.models[n].update(vals)
        self.dirty.add(n)
        self.loading = True
        for k in MODEL_KEYS:
            self.edits[k].setText(self.models[n].get(k, ''))
        self.loading = False
        for k, v in vals.items():
            if v:
                self.pending['lidar%d_%s' % (n, k)] = v
        if self.chk_live.isChecked():
            self.live_timer.start(150)
        self.relabel(n)
        self.update_info()
        self.shadow = None
        self.schedule_shadow()
        self.status.setText('lidar%d = %s (reinicie viewer_3D/lidar%d/gazebo para valer)' % (n, name, n))

    def copy_model(self):
        src = self.copy_src.currentData()
        if src == self.cur:
            return
        self.models[self.cur] = dict(self.models[src])
        self.dirty.add(self.cur)
        for k in MODEL_KEYS:
            self.edits[k].setText(self.models[self.cur].get(k, ''))
            if self.models[self.cur].get(k):
                self.pending['lidar%d_%s' % (self.cur, k)] = self.models[self.cur][k]
        self.relabel(self.cur)
        if self.chk_live.isChecked():
            self.live_timer.start(150)
        self.update_info()
        self.schedule_shadow()
        self.status.setText('modelo do lidar%d copiado do lidar%d (reinicie viewer_3D/lidar%d)' % (self.cur, src, self.cur))

    def model_edited(self, k):
        if self.loading or self.cur is None:
            return
        v = self.edits[k].text().strip()
        if v == self.models[self.cur].get(k, ''):
            return
        self.models[self.cur][k] = v
        self.dirty.add(self.cur)
        if v:
            self.pending['lidar%d_%s' % (self.cur, k)] = v
            if self.chk_live.isChecked():
                self.live_timer.start(150)
        if k == 'model':
            self.relabel(self.cur)
        if k == 'vertical_angles':
            self.schedule_shadow()
        self.update_info()

    def label(self, n):
        return 'lidar%d (%s)%s' % (n, (self.models[n].get('model') or '?').split()[0],
                                   '  [garra]' if self.links.get(n) else '')

    def relabel(self, n):
        for i in range(self.list.count()):
            it = self.list.item(i)
            if it.data(Qt.UserRole) == n:
                it.setText(self.label(n))

    def alpha_changed(self, i):
        self.alpha = (1.0, 0.5, 0.2)[i]
        self.view.update()

    def focus_lidar(self):
        if self.cur is not None:
            self.view.target = self.lidar_tf(self.cur)[:3, 3].copy()
            self.view.dist = 3.0
            self.view.update()

    # ---------------------------------------------------------------- sombra
    def schedule_shadow(self):
        if self.chk_auto.isChecked() and self.chk_rays.isChecked():
            self.shadow_timer.start(350)

    def compute_shadow(self):
        if self.cur is None or self.tris is None:
            return
        o, d = self.ray_dirs(self.cur)
        t = ray_mesh(o, d, self.v0, self.e1, self.e2)
        self.shadow = t
        L = self.spin_ray_len.value()
        blocked = np.isfinite(t)
        near = t < 1.0
        self.shadow_label.setText('lidar%d: %d raios, %.0f%% batem na empilhadeira (%.0f%% a menos de 1 m)' % (
            self.cur, len(t), 100.0 * blocked.mean(), 100.0 * near.mean()))
        self.view.update()

    # ---------------------------------------------------------------- param_daemon ao vivo
    def flush_live(self):
        if not self.pending or not self.chk_live.isChecked():
            return
        items, self.pending = list(self.pending.items()), {}
        for name, value in items:
            self.param_set(name, value)

    def param_set(self, name, value):
        """param_tool set grava e fica no IPC_dispatch() para sempre, com o printf preso no buffer:
        stdbuf -oL solta a linha de confirmacao, e o processo e' encerrado ao le-la (ou em 3 s)."""
        proc = QProcess(self)
        proc.setWorkingDirectory(os.path.dirname(PARAM_TOOL))
        proc.setProcessChannelMode(QProcess.MergedChannels)
        buf = {'out': ''}

        def read():
            buf['out'] += bytes(proc.readAll()).decode(errors='replace')
            if (name + ' = ') in buf['out']:
                proc.kill()
                self.live_fail = 0
                self.status.setText('ao vivo: %s = %s' % (name, value))

        def timeout():
            if proc.state() != QProcess.NotRunning:
                proc.kill()
                self.live_fail += 1
                self.status.setText('ao vivo: %s sem resposta (param_daemon/central no ar?)' % name)
                if self.live_fail >= 3:
                    self.chk_live.setChecked(False)
                    self.status.setText('ao vivo DESLIGADO: o param_daemon nao responde. Edite e salve no .ini; '
                                        'religue a caixa com o sistema no ar.')
        proc.readyRead.connect(read)
        QTimer.singleShot(3000, timeout)
        proc.start('stdbuf', ['-oL', PARAM_TOOL, 'set', name, value])

    # ---------------------------------------------------------------- salvar
    def save_ini(self):
        if not self.dirty:
            self.status.setText('nada alterado')
            return
        if not self.backup_done:
            bak = self.ini_path + '.bak-lidar_editor'
            if not os.path.exists(bak):
                shutil.copy2(self.ini_path, bak)
            self.backup_done = True
        text = open(self.ini_path, errors='replace').read()
        report = []
        for n in sorted(self.dirty):
            vals = {k: '%.6f' % v for k, v in zip(POSE_KEYS, self.poses[n])}
            vals.update({k: v for k, v in self.models[n].items() if v})
            # o astro le' shot_size valores de ray_order e de vertical_angles: ray_order fora do tamanho
            # vira 0..N-1; vertical_angles com outra quantidade e' so' avisado
            try:
                ns = int(float(vals.get('shot_size', '0')))
            except ValueError:
                ns = 0
            if ns > 0:
                if len(vals.get('ray_order', '').split()) != ns:
                    vals['ray_order'] = ' '.join(str(i) for i in range(ns))
                    self.models[n]['ray_order'] = vals['ray_order']
                nv = len(vals.get('vertical_angles', '').split())
                if nv != ns:
                    QMessageBox.warning(self, 'lidar%d' % n, 'shot_size %d mas vertical_angles tem %d valores:'
                                        ' o astro vai ler errado. Corrija antes de usar.' % (ns, nv))
            if self.links.get(n):
                vals['link'] = self.links[n]
            changed = []
            # link desligado ou horizontal_angles apagado: comenta a linha que houver no .ini
            for k in ('link', 'horizontal_angles'):
                if k not in vals:
                    name = 'lidar%d_%s' % (n, k)
                    new = re.sub(r'^(%s\b[^\n]*)$' % re.escape(name), r'#lidar_pose_editor \1', text, flags=re.M)
                    if new != text:
                        text = new
                        changed.append(k + '(comentado)')
            for k, val in vals.items():
                name = 'lidar%d_%s' % (n, k)
                pat = re.compile(LINE_RE % re.escape(name), re.M)
                m = pat.search(text)
                if m:
                    if m.group(3).strip() != val:
                        text = text[:m.start()] + m.group(1) + m.group(2) + val + m.group(4) + text[m.end():]
                        changed.append(k)
                else:
                    last = None
                    for last in re.finditer(r'^lidar%d_[^\n]*$' % n, text, re.M):
                        pass
                    pos = last.end() if last else len(text)
                    text = text[:pos] + '\n%s\t\t%s\t# EMPILHADEIRA: lidar_pose_editor' % (name, val) + text[pos:]
                    changed.append(k + '(novo)')
            report.append('lidar%d: %s' % (n, ' '.join(changed) or 'igual'))
        open(self.ini_path, 'w').write(text)
        saved = sorted(self.dirty)
        self.dirty.clear()
        self.values = read_ini_values(self.ini_path)
        self.update_info()
        self.status.setText('Salvo. ' + ' | '.join(report) + '. Reinicie "gazebo" e ' +
                            ', '.join('"lidar%d"' % n for n in saved) + ' no proccontrol_gui.')

    def closeEvent(self, ev):
        if self.dirty and QMessageBox.question(
                self, 'Sair', 'Ha LiDARs alterados e nao salvos (%s). Sair mesmo assim?' %
                ', '.join('lidar%d' % n for n in sorted(self.dirty))) != QMessageBox.Yes:
            ev.ignore()
            return
        ev.accept()


def main():
    app = QApplication(sys.argv)
    ini = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_INI)
    if not os.path.isfile(ini):
        sys.exit('nao achei ' + ini)
    w = Editor(ini)
    w.resize(1500, 900)
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
