#!/usr/bin/env python3
"""Mundo "port": o big_depot com a frente sul aberta para um patio de porto de celulose (como a foto
do local do projeto): cobertura avancando sobre o patio, faixa de pedestre vermelha, patio de concreto
(textura gerada, worlds/meshes/port_concrete.png, placas de 6 m), cais com meio-fio e cabecos, agua (textura gerada, worlds/meshes/port_water.png),
navio graneleiro com guindastes de bordo (modelo "Ship AAA" baixado pelo Miguel), caminhoes (poly.pizza, CC BY 3.0,
worlds/models/ATTRIBUTION.txt), unidades de celulose de 2 t com textura (worlds/models/pulp_unit), cones,
placas e postes; o resto sao primitivas.

    make_port_world.py [--depot worlds/big_depot.sdf] [--out worlds/port.sdf]

Referencial do mundo (o mesmo do galpao): x ao longo do cais, y para dentro do galpao. O galpao vai de
x -15,8 a 97,4 e y -13,4 a 65,4; a frente sul (y -13,4) fica aberta, so' com colunas.
"""

import argparse
import math
import os
import re

from world_uris import leftovers, portable

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.abspath(os.path.join(HERE, '..', 'worlds'))
MESHES = os.path.join(WORLDS, 'meshes')
MODELS = os.path.join(WORLDS, 'models')        # modelos baixados (ver models/ATTRIBUTION.txt)
DEPOT_TEX = os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/depot/6/materials/textures')

# galpao (big_depot.sdf)
GX0, GX1, GY0, GY1, GH = -15.8, 97.4, -13.4, 65.4, 12.0
# patio / cais
APRON_X0, APRON_X1 = -45.0, 145.0
CANOPY_Y = GY0 - 12.0          # beirada da cobertura
LANE_Y0, LANE_Y1 = CANOPY_Y - 0.5, CANOPY_Y - 3.0   # faixa de pedestre vermelha
QUAY_Y = GY0 - 48.0            # beira do cais
SHIP_Y0, SHIP_BEAM = QUAY_Y - 3.0, 28.0
SHIP_X0, SHIP_X1 = -30.0, 150.0

COLORS = {
    'concrete': (0.62, 0.62, 0.60), 'concrete_dark': (0.50, 0.50, 0.49), 'wall': (0.72, 0.73, 0.72),
    'roof': (0.55, 0.56, 0.58), 'column': (0.40, 0.40, 0.40), 'red_lane': (0.70, 0.22, 0.20),
    'yellow': (0.95, 0.78, 0.05), 'white': (0.95, 0.95, 0.95), 'bale': (0.93, 0.93, 0.90),
    'water': (0.10, 0.30, 0.45), 'hull_blue': (0.05, 0.35, 0.75), 'hull_red': (0.65, 0.15, 0.12),
    'superstructure': (0.95, 0.95, 0.93), 'deck': (0.35, 0.40, 0.35), 'black': (0.08, 0.08, 0.08),
    'orange': (1.0, 0.45, 0.05), 'truck_cab': (0.90, 0.90, 0.92), 'trailer': (0.25, 0.25, 0.27),
    'trees': (0.12, 0.30, 0.12), 'grey': (0.45, 0.45, 0.47), 'crane': (0.95, 0.75, 0.10),
}


class Builder:
    def __init__(self):
        self.parts = []
        self.models = []       # modelos proprios (fardos dinamicos), fora do modelo estatico "port"
        self.n = 0

    def _mat(self, color, emissive=False):
        r, g, b = COLORS[color] if isinstance(color, str) else color
        e = '<emissive>%.2f %.2f %.2f 1</emissive>' % (r * 0.5, g * 0.5, b * 0.5) if emissive else ''
        return ('<material><ambient>%.3f %.3f %.3f 1</ambient><diffuse>%.3f %.3f %.3f 1</diffuse>'
                '<specular>0.1 0.1 0.1 1</specular>%s</material>' % (r * 0.8, g * 0.8, b * 0.8, r, g, b, e))

    def box(self, name, x, y, z, sx, sy, sz, color, collide=True, roll=0.0, pitch=0.0, yaw=0.0, emissive=False):
        """caixa com centro (x, y, z)"""
        self.n += 1
        nm = '%s_%d' % (name, self.n)
        pose = '<pose>%.3f %.3f %.3f %.4f %.4f %.4f</pose>' % (x, y, z, roll, pitch, yaw)
        geo = '<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>' % (sx, sy, sz)
        s = '<visual name="%s_v">%s%s%s</visual>' % (nm, pose, geo, self._mat(color, emissive))
        if collide:
            s += '<collision name="%s_c">%s%s</collision>' % (nm, pose, geo)
        self.parts.append(s)

    def cyl(self, name, x, y, z, r, h, color, collide=True, roll=0.0, pitch=0.0, yaw=0.0):
        """cilindro com centro (x, y, z), eixo z (gire com roll/pitch)"""
        self.n += 1
        nm = '%s_%d' % (name, self.n)
        pose = '<pose>%.3f %.3f %.3f %.4f %.4f %.4f</pose>' % (x, y, z, roll, pitch, yaw)
        geo = '<geometry><cylinder><radius>%.3f</radius><length>%.3f</length></cylinder></geometry>' % (r, h)
        s = '<visual name="%s_v">%s%s%s</visual>' % (nm, pose, geo, self._mat(color))
        if collide:
            s += '<collision name="%s_c">%s%s</collision>' % (nm, pose, geo)
        self.parts.append(s)

    def tex_quad(self, name, corners, texture, tile, color='white'):
        """retangulo com textura repetida a cada 'tile' m (malha OBJ propria em worlds/meshes);
        corners: 4 cantos (x, y, z) na ordem do contorno; desenha os dois lados"""
        import numpy as np
        c = [np.array(v, float) for v in corners]
        u_len, v_len = np.linalg.norm(c[1] - c[0]), np.linalg.norm(c[3] - c[0])
        path = os.path.join(MESHES, 'port_%s.obj' % name)
        uv = [(0, 0), (u_len / tile, 0), (u_len / tile, v_len / tile), (0, v_len / tile)]
        with open(path, 'w') as f:
            f.write('# GERADO por make_port_world.py\n')
            for v in c:
                f.write('v %.3f %.3f %.3f\n' % tuple(v))
            for a, b_ in uv:
                f.write('vt %.4f %.4f\n' % (a, b_))
            n = np.cross(c[1] - c[0], c[3] - c[0]); n /= np.linalg.norm(n)
            f.write('vn %.4f %.4f %.4f\nvn %.4f %.4f %.4f\n' % (tuple(n) + tuple(-n)))
            f.write('f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\nf 1/1/2 3/3/2 2/2/2\nf 1/1/2 4/4/2 3/3/2\n')
        self.n += 1
        r, g, b_ = COLORS[color] if isinstance(color, str) else color
        self.parts.append(
            '<visual name="%s_%d_v"><geometry><mesh><uri>file://%s</uri></mesh></geometry>'
            '<material><diffuse>%.2f %.2f %.2f 1</diffuse><specular>0.05 0.05 0.05 1</specular>'
            '<pbr><metal><albedo_map>%s</albedo_map><metalness>0.0</metalness><roughness>0.9</roughness></metal></pbr>'
            '</material></visual>' % (name, self.n, path, r, g, b_, texture))

    def glb(self, name, uri, x, y, z, scale, roll=math.pi / 2, pitch=0.0, yaw=math.pi / 2, collision=None):
        """modelo .glb (Y para cima): roll pi/2 poe o Y do modelo no Z do mundo, yaw pi/2 a frente (+Z do
        modelo) em +x. collision: (cx, cy, cz, sx, sy, sz) caixa no mundo, opcional"""
        self.n += 1
        self.parts.append('<visual name="%s_%d_v"><pose>%.3f %.3f %.3f %.4f %.4f %.4f</pose><geometry><mesh>'
                          '<uri>file://%s</uri><scale>%.3f %.3f %.3f</scale></mesh></geometry></visual>'
                          % (name, self.n, x, y, z, roll, pitch, yaw, uri, *scale))
        if collision:
            cx, cy, cz, sx, sy, sz = collision
            self.parts.append('<collision name="%s_%d_c"><pose>%.3f %.3f %.3f 0 0 %.4f</pose><geometry><box>'
                              '<size>%.3f %.3f %.3f</size></box></geometry></collision>'
                              % (name, self.n, cx, cy, cz, yaw - math.pi / 2, sx, sy, sz))

    def mesh(self, name, uri, x, y, z, yaw=0.0, scale=1.0, roll=0.0):
        """malha (OBJ/DAE), so' visual; roll pi/2 para malha com Y para cima"""
        self.n += 1
        self.parts.append('<visual name="%s_%d_v"><pose>%.3f %.3f %.3f %.4f 0 %.4f</pose><geometry><mesh>'
                          '<uri>file://%s</uri><scale>%.4f %.4f %.4f</scale></mesh></geometry></visual>'
                          % (name, self.n, x, y, z, roll, yaw, uri, scale, scale, scale))

    def collision_box(self, name, x, y, z, sx, sy, sz, yaw=0.0):
        self.n += 1
        self.parts.append('<collision name="%s_%d_c"><pose>%.3f %.3f %.3f 0 0 %.4f</pose><geometry><box>'
                          '<size>%.3f %.3f %.3f</size></box></geometry></collision>' % (name, self.n, x, y, z, yaw, sx, sy, sz))

    def flat(self, name, x0, y0, x1, y1, color, z=0.004):
        """faixa pintada no chao (so' visual)"""
        self.box(name, (x0 + x1) / 2, (y0 + y1) / 2, z, abs(x1 - x0), abs(y1 - y0), 0.004, color, collide=False)


# Unidade de celulose (ficha tecnica UPM Euca, BEKP): 8 fardos de 250 kg (94,5 x 69 x 45 cm, 3 arames)
# = 2 t, 0,945 (L) x 1,39 (W) x 1,80 m (H), 7 arames de 3 mm. Modelo: worlds/models/pulp_unit (textura gerada).
UNIT_L, UNIT_W, UNIT_H = 0.945, 1.39, 1.80
PULP_UNIT = os.path.join(MODELS, 'pulp_unit', 'pulp_unit.obj')


def bale_block(b, x0, y0, nx, ny, levels, gap=0.06):
    """bloco de nx (ao longo de x) por ny (ao longo de y) unidades, 'levels' de altura; canto (x0, y0).
    Uma caixa de colisao por bloco."""
    for i in range(nx):
        for j in range(ny):
            for k in range(levels):
                b.mesh('pulp', PULP_UNIT, x0 + UNIT_L / 2 + i * (UNIT_L + gap), y0 + UNIT_W / 2 + j * (UNIT_W + gap),
                       k * (UNIT_H + 0.01))
    sx, sy, sz = nx * (UNIT_L + gap) - gap, ny * (UNIT_W + gap) - gap, levels * (UNIT_H + 0.01)
    b.collision_box('pulp_block', x0 + sx / 2, y0 + sy / 2, sz / 2, sx, sy, sz)


UNIT_MASS = 2000.0
UNIT_MU = 0.6             # celulose em concreto / celulose em celulose


def bale_dynamic(b, x0, y0, nx, ny, levels, gap=0.06):
    """como bale_block, mas cada unidade e' um modelo com fisica (2 t, caixa de colisao) que a garra pega:
    pulp_dyn_<n>, link "link", origem no centro da base. O sim.launch.py le' estes nomes no .sdf e poe
    na Hyster uma DetachableJoint por unidade; o can_bridge prende/solta (ver o README)."""
    m = UNIT_MASS
    ixx = m / 12 * (UNIT_W ** 2 + UNIT_H ** 2)
    iyy = m / 12 * (UNIT_L ** 2 + UNIT_H ** 2)
    izz = m / 12 * (UNIT_L ** 2 + UNIT_W ** 2)
    for i in range(nx):
        for j in range(ny):
            for k in range(levels):
                name = 'pulp_dyn_%d' % len(b.models)
                x = x0 + UNIT_L / 2 + i * (UNIT_L + gap)
                y = y0 + UNIT_W / 2 + j * (UNIT_W + gap)
                z = k * (UNIT_H + 0.005)
                b.models.append(
                    '<model name="%s"><pose>%.3f %.3f %.3f 0 0 0</pose><link name="link">'
                    '<inertial><pose>0 0 %.3f 0 0 0</pose><mass>%.1f</mass><inertia><ixx>%.1f</ixx><ixy>0</ixy><ixz>0</ixz>'
                    '<iyy>%.1f</iyy><iyz>0</iyz><izz>%.1f</izz></inertia></inertial>'
                    '<collision name="c"><pose>0 0 %.3f 0 0 0</pose><geometry><box><size>%.3f %.3f %.3f</size></box></geometry>'
                    '<surface><friction><ode><mu>%.2f</mu><mu2>%.2f</mu2></ode></friction></surface></collision>'
                    '<visual name="v"><geometry><mesh><uri>file://%s</uri></mesh></geometry></visual>'
                    '</link></model>'
                    % (name, x, y, z, UNIT_H / 2, m, ixx, iyy, izz, UNIT_H / 2, UNIT_L, UNIT_W, UNIT_H,
                       UNIT_MU, UNIT_MU, PULP_UNIT))


def truck(b, x, y, yaw=0.0):
    """caminhao (cavalo + bau, "Cargo Truck" de J-Toastie, CC BY 3.0) com ~16,5 x 2,5 x 4,0 m;
    o modelo tem 6,68 (Z) x 1,68 (X) x 2,42 (Y) e a frente em +Z"""
    b.glb('truck', os.path.join(MODELS, 'cargo_truck.glb'), x, y, 0.0, (1.5, 1.65, 2.47), yaw=yaw + math.pi / 2,
          collision=(x, y, 2.0, 16.5, 2.5, 4.0))


SHIP_SCALE = 7.7          # o modelo tem 23,4 m de comprimento -> ~180 m
SHIP_Z = -1.0             # afunda um pouco o casco: o costado (branco no modelo) fica acima da agua


def ship(b):
    """graneleiro com guindastes de bordo ("Ship AAA", baixado pelo Miguel; worlds/models/ship_aaa, .dae com Z
    para cima): x = comprimento (popa em -x), ~180 x 31 m com escala 7,7"""
    L = 23.39 * SHIP_SCALE
    beam = 4.02 * SHIP_SCALE
    cx = (SHIP_X0 + SHIP_X1) / 2
    cy = SHIP_Y0 - beam / 2
    # ship_aaa.obj: o OBJ do modelo (Y para cima) com DUPLA FACE nas pecas grandes cujas faces estavam viradas
    # para dentro (casco/costado: de fora via-se o interior do navio) -- ver models/ship_aaa/LEIA-ME.txt
    b.mesh('ship', os.path.join(MODELS, 'ship_aaa', 'ship_aaa.obj'), cx - 0.565 * SHIP_SCALE, cy, SHIP_Z, 0.0,
           SHIP_SCALE, roll=math.pi / 2)
    b.collision_box('ship_hull', cx, cy, 6.0, L, beam, 12.0)


def build():
    b = Builder()
    # ---------------- galpao: paredes N/L/O em caixa (a malha das paredes sai) e frente sul aberta
    wall_tex = os.path.join(DEPOT_TEX, 'WALLS_Albedo.png')
    b.box('wall_n', (GX0 + GX1) / 2, GY1, GH / 2, GX1 - GX0, 0.3, GH, 'wall')
    b.box('wall_w', GX0, (GY0 + GY1) / 2, GH / 2, 0.3, GY1 - GY0, GH, 'wall')
    b.box('wall_e', GX1, (GY0 + GY1) / 2, GH / 2, 0.3, GY1 - GY0, GH, 'wall')
    for nm, (xa, ya, xb, yb) in (('wall_n_tex', (GX0, GY1 - 0.16, GX1, GY1 - 0.16)),
                                 ('wall_w_tex', (GX0 + 0.16, GY0, GX0 + 0.16, GY1)),
                                 ('wall_e_tex', (GX1 - 0.16, GY0, GX1 - 0.16, GY1))):
        b.tex_quad(nm, [(xa, ya, 0.0), (xb, yb, 0.0), (xb, yb, GH), (xa, ya, GH)], wall_tex, 6.0, 'white')
    nfront = 12
    for i in range(nfront + 1):
        x = GX0 + i * (GX1 - GX0) / nfront
        b.box('front_column', x, GY0, GH / 2, 0.6, 0.6, GH, 'column')
    b.box('front_beam', (GX0 + GX1) / 2, GY0, GH - 0.75, GX1 - GX0, 0.5, 1.5, 'wall')
    # cobertura sobre o patio + colunas na beirada
    b.box('canopy', (GX0 + GX1) / 2, (GY0 + CANOPY_Y) / 2, GH + 0.6, GX1 - GX0 + 1.0, GY0 - CANOPY_Y + 0.5, 0.4, 'roof', collide=False)
    for i in range(7):
        x = GX0 + i * (GX1 - GX0) / 6
        b.box('canopy_column', x, CANOPY_Y, GH / 2 + 0.3, 0.5, 0.5, GH + 0.6, 'column')

    # ---------------- patio
    b.tex_quad('apron', [(APRON_X0, QUAY_Y, 0.002), (APRON_X1, QUAY_Y, 0.002), (APRON_X1, GY0, 0.002),
                         (APRON_X0, GY0, 0.002)], os.path.join(MESHES, 'port_concrete.png'), 6.0, 'white')
    b.flat('apron_shade', GX0, CANOPY_Y, GX1, GY0, 'concrete_dark', z=0.003)
    b.flat('red_lane', APRON_X0, LANE_Y1, APRON_X1, LANE_Y0, 'red_lane')
    b.flat('lane_edge', APRON_X0, LANE_Y0 - 0.15, APRON_X1, LANE_Y0, 'yellow', z=0.005)
    b.flat('lane_edge', APRON_X0, LANE_Y1, APRON_X1, LANE_Y1 + 0.15, 'yellow', z=0.005)
    for k in range(int((APRON_X1 - APRON_X0) / 6)):       # tracejados
        x = APRON_X0 + k * 6
        b.flat('dash_y', x, QUAY_Y + 20.0, x + 3, QUAY_Y + 20.15, 'yellow', z=0.005)
        b.flat('dash_w', x, QUAY_Y + 9.0, x + 2, QUAY_Y + 9.2, 'white', z=0.005)
        b.flat('dash_r', x, QUAY_Y + 26.0, x + 1.5, QUAY_Y + 26.25, 'red_lane', z=0.005)
        b.flat('dash_w2', x + 1.5, QUAY_Y + 26.0, x + 3, QUAY_Y + 26.25, 'white', z=0.005)
    # cais: meio-fio amarelo (colisao: nao deixa cair na agua) e cabecos
    b.box('quay_curb', (APRON_X0 + APRON_X1) / 2, QUAY_Y + 0.25, 0.15, APRON_X1 - APRON_X0, 0.5, 0.3, 'yellow')
    for k in range(int((APRON_X1 - APRON_X0) / 15) + 1):
        x = APRON_X0 + 5 + k * 15
        b.cyl('bollard', x, QUAY_Y + 1.2, 0.35, 0.3, 0.7, 'black')
        b.cyl('bollard_cap', x, QUAY_Y + 1.2, 0.75, 0.38, 0.12, 'yellow')
    # agua
    wx0, wx1, wy0, wy1 = APRON_X0 - 200.0, APRON_X1 + 200.0, QUAY_Y - 240.0, QUAY_Y
    b.tex_quad('water', [(wx0, wy0, 0.003), (wx1, wy0, 0.003), (wx1, wy1, 0.003), (wx0, wy1, 0.003)],
               os.path.join(MESHES, 'port_water.png'), 12.0, 'white')
    # margem com arvores ao fundo
    b.box('trees', 60.0, QUAY_Y - 230.0, 7.0, 700.0, 20.0, 14.0, 'trees', collide=False)

    # ---------------- navio
    ship(b)
    # TESTE da garra: 6 unidades COM FISICA, uma ao lado da outra com 1 m de folga (os bracos passam entre
    # elas). Com 72 a fisica levava o Gazebo a ~0,7 do tempo real sozinho.
    bale_dynamic(b, SHIP_X0 + 40.0, QUAY_Y + 4.0, 6, 1, 1, gap=1.0)
    # caminhoes no patio
    truck(b, 5.0, QUAY_Y + 14.0, yaw=0.0)
    truck(b, 55.0, QUAY_Y + 15.0, yaw=0.0)
    # cones, placas amarelas e postes
    for k in range(10):
        b.cyl('cone', 20.0 + k * 1.2, LANE_Y1 - 1.0, 0.35, 0.18, 0.7, 'orange')
    for x in (12.0, 40.0, 70.0, 100.0):
        b.box('sign_post', x, LANE_Y1 - 1.5, 0.6, 0.06, 0.06, 1.2, 'grey')
        b.box('sign', x, LANE_Y1 - 1.5, 1.35, 1.0, 0.04, 0.5, 'yellow')
    for k in range(7):
        x = APRON_X0 + 10 + k * 30
        b.cyl('light_pole', x, QUAY_Y + 30.0, 10.0, 0.18, 20.0, 'grey')
        b.box('light_head', x, QUAY_Y + 30.0, 20.2, 1.6, 0.8, 0.4, 'white', collide=False, emissive=True)
    return b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--depot', default=os.path.join(HERE, '..', 'worlds', 'big_depot.sdf'))
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'worlds', 'port.sdf'))
    a = ap.parse_args()
    t = open(a.depot).read()
    # malhas das paredes/janelas saem (a frente sul vira aberta); a colisao da parede sul (wall0) tambem
    for name in ('WALLS_visual', 'WINDOW_visual'):
        t, n = re.subn(r'\s*<visual name="%s">.*?</visual>' % name, '', t, flags=re.S)
        assert n >= 1, name
    t, n = re.subn(r'\s*<collision name="wall[0-3]">.*?</collision>', '', t, flags=re.S)
    assert n == 4
    t = t.replace('<world name="big_depot">', '<world name="port">')
    t = t.replace('<size>340 340</size>', '<size>900 900</size>')
    b = build()
    port = ('\n    <model name="port">\n      <static>true</static>\n      <link name="main">\n        '
            + '\n        '.join(b.parts) + '\n      </link>\n    </model>\n')
    port += ''.join('    %s\n' % m for m in b.models)
    t = t.replace('  </world>', port + '  </world>')
    t = re.sub(r'<!--.*?-->', '''<!--
  Porto de celulose para a simulacao da empilhadeira: o big_depot com a frente sul aberta para o patio
  (cobertura, faixa de pedestre, patio, cais, navio com guindastes, caminhoes, cones, postes).
  GERADO por tools/make_port_world.py a partir do big_depot.sdf -- edite o gerador, nao este arquivo.
  Galpao: pecas do "Depot" (OpenRobotics) e colisoes do "tugbot_depot" (MovAi), Gazebo Fuel, CC-BY 4.0.
  Porto: navio e caminhao de poly.pizza (CC BY 3.0, worlds/models/ATTRIBUTION.txt), resto primitivas. Inicio sugerido no patio: x 5, y -27, yaw 0 (ou dentro do galpao: 0.2, 8.8).
-->''', t, count=1, flags=re.S)
    t = portable(t, os.path.dirname(os.path.abspath(a.out)))
    for p in leftovers(t):
        print('AVISO: caminho absoluto no mundo:', p)
    open(a.out, 'w').write(t)
    print('%s: %d pecas no porto, %d unidades de celulose com fisica' % (os.path.abspath(a.out), b.n, len(b.models)))


if __name__ == '__main__':
    main()
