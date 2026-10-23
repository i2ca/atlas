#!/usr/bin/env python3
"""Gera worlds/big_depot.sdf: o galpao "Depot" (OpenRobotics, Gazebo Fuel, CC-BY 4.0)
bem maior, para a empilhadeira.

O Depot e' uma malha so', dividida em partes (paredes, teto, chao, pilares, prateleiras com
caixas...). Aqui:
  - a CASCA (paredes, teto, chao, janelas) e' esticada para cercar tudo;
  - o MIOLO do galpao original (prateleiras e caixas; sem pilares e ventiladores) vira um
    bloco que se repete em grade, no tamanho original, com corredores largos entre os blocos
    (os corredores dentro de cada bloco continuam com ~1 m, como no original); as pecas
    presas nas paredes do original (tubos, ventiladores de parede) ficam de fora;
  - as colisoes de cada bloco sao as do mundo "tugbot_depot" (MovAi, CC-BY 4.0); as paredes
    sao novas, no tamanho da casca.

Precisa do Depot no cache do Gazebo:
  gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot

Uso: tools/make_big_depot.py [--tiles-x 4] [--tiles-y 4] [--out worlds/big_depot.sdf]
"""

import argparse
import glob
import os
import re

import numpy as np

from world_uris import leftovers, portable

HERE = os.path.dirname(os.path.abspath(__file__))
FUEL = os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org')

# Dimensoes do galpao original (frame do modelo Depot, escala 0.6)
ORIG_X = 30.17            # comprimento entre as paredes
ORIG_Y = 14.90
ORIG_Y_CENTER = -0.16     # centro das paredes em y
ORIG_HEIGHT = 9.0
ORIG_SCALE = 0.6

# Miolo: o que esta' entre x -7.8 (pilares) e 12 (ultima prateleira), y -5.4..5.8.
# A faixa de x -15..-8 do original fica livre: vira o corredor entre blocos.
CORE_MIN_X, CORE_MAX_X = -7.8, 12.0
CORE_MIN_Y, CORE_MAX_Y = -5.4, 5.8

SHELL_PARTS = ['ROOF_visual', 'WALLS_visual', 'WINDOW_visual', 'FLOOR_visual']
# Pilares/vigas (PILLERS) e ventiladores (FANS) ficam de fora: no galpao grande eles viravam
# barras de ferro no meio dos corredores e tiravam espaco da empilhadeira (pedido de 19/10).
CORE_PARTS = ['BOXSET_visual', 'crates']
# Aquecedores do teto (FANS, 6-10 m): sobem junto com o teto. Ficam de fora as pecas presas
# nas paredes do original (tubos, ventiladores de parede FanBack/FAN1/FAN2, alarme, interfone):
# repetidas no meio do galpao elas ficariam soltas no ar.
CEILING_PARTS = []


def find_depot():
    dirs = sorted(glob.glob(os.path.join(FUEL, 'openrobotics/models/depot/*/')))
    if not dirs:
        raise SystemExit('Depot nao esta no cache. Rode:\n  gz fuel download -u '
                         'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot')
    return dirs[-1]


def find_collisions():
    files = glob.glob(os.path.join(FUEL, 'movai/worlds/tugbot_depot/*/tugbot_depot.sdf'))
    if not files:
        raise SystemExit('tugbot_depot nao esta no cache. Rode:\n  gz fuel download -u '
                         'https://fuel.gazebosim.org/1.0/MovAi/worlds/tugbot_depot')
    s = open(sorted(files)[-1]).read()
    block = s[s.index("<model name='depot_collision'>"):]
    block = block[:block.index('</model>')]
    boxes = []
    for m in re.finditer(r"<collision name='([^']*)'>\s*<pose>([^<]*)</pose>.*?<size>([^<]*)</size>", block, re.S):
        name = m.group(1)
        if name.startswith('wall') or name.startswith('pilar') or name == 'stairs':
            continue
        boxes.append((name, [float(v) for v in m.group(2).split()], m.group(3).split()))
    return boxes


# Pecas do BOXSET encostadas nas paredes do original (ventiladores de parede, quadros...):
# no meio do galpao grande ficariam soltas no ar. Fica so' o que esta' na faixa das
# prateleiras (frame do modelo, metros).
BOXSET_KEEP_X = (-13.5, 13.5)
BOXSET_KEEP_Y = (-6.8, 6.5)


def make_boxset_mesh(depot, out_path):
    """Copia a malha do Depot so' com o BOXSET, sem as pecas de parede."""
    dae = open(os.path.join(depot, 'meshes', 'Depot.dae')).read()
    geo = re.search(r'<geometry id="BOXSET-lib" name="BOXSETMesh">.*?</geometry>', dae, re.S)
    g = geo.group(0)
    pos = np.array(re.search(r'<float_array id="BOXSET-POSITION-array"[^>]*>([^<]*)<', g).group(1).split(),
                   float).reshape(-1, 3)
    tri = re.search(r'<triangles count="(\d+)"([^>]*)>(.*?)</triangles>', g, re.S)
    stride = len(re.findall(r'<input ', tri.group(3)))
    p_text = re.search(r'<p>([^<]*)</p>', tri.group(3)).group(1)
    p = np.array(p_text.split(), np.int64).reshape(-1, 3, stride)
    vid = p[:, :, 0]
    node = re.search(r'<node[^>]*name="BOXSET"[^>]*>\s*<matrix[^>]*>([^<]*)</matrix>', dae)
    m = np.array(node.group(1).split(), float).reshape(4, 4)
    world = (np.c_[pos, np.ones(len(pos))] @ m.T)[:, :3] * ORIG_SCALE / 100.0

    # ilhas de triangulos conectados (union-find nos vertices)
    parent = np.arange(len(pos))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for t in vid:
        r = find(t[0])
        for v in t[1:]:
            parent[find(v)] = r
    roots = np.array([find(t[0]) for t in vid])
    centers = {}
    for r in np.unique(roots):
        vv = world[vid[roots == r].ravel()]
        centers[r] = (vv.min(0) + vv.max(0)) / 2.0
    keep = np.array([BOXSET_KEEP_X[0] <= centers[r][0] <= BOXSET_KEEP_X[1] and
                     BOXSET_KEEP_Y[0] <= centers[r][1] <= BOXSET_KEEP_Y[1] for r in roots])

    new_p = ' '.join(map(str, p[keep].ravel()))
    new_tri = tri.group(0).replace(tri.group(3).split('<p>')[0] + '<p>' + p_text + '</p>',
                                   tri.group(3).split('<p>')[0] + '<p>' + new_p + '</p>')
    new_tri = new_tri.replace('count="%s"' % tri.group(1), 'count="%d"' % keep.sum(), 1)
    # so' a geometria e o no do BOXSET
    out = re.sub(r'<library_geometries>.*?</library_geometries>',
                 '<library_geometries>\n' + g.replace(tri.group(0), new_tri) + '\n</library_geometries>', dae,
                 flags=re.S)
    out = re.sub(r'<node [^>]*name="(?!BOXSET")[^"]*".*?</node>\s*', '', out, flags=re.S)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    open(out_path, 'w').write(out)
    return int((~keep).sum()), len(roots)


def read_visuals(depot):
    """Devolve {nome: [(xml, (dx, dy, dz)), ...]} com os caminhos ja' absolutos."""
    sdf = open(os.path.join(depot, 'model.sdf')).read()
    visuals = {}
    for link in re.finditer(r'<link name="([^"]*)">(.*?)</link>', sdf, re.S):
        link_body = link.group(2)
        lp = re.match(r'\s*<pose>([^<]*)</pose>', link_body)
        link_pose = [float(v) for v in lp.group(1).split()[:3]] if lp else [0.0, 0.0, 0.0]
        for v in re.finditer(r'<visual name="([^"]*)">(.*?)</visual>', link_body, re.S):
            name, body = v.group(1), v.group(2)
            if name == 'visual':
                sub = re.search(r'<submesh>\s*<name>([^<]*)</name>', body)
                name = 'WALLFAN_visual' if sub and sub.group(1).startswith('FAN') else name
            vp = re.search(r'<pose>([^<]*)</pose>', body)
            vis_pose = [float(x) for x in vp.group(1).split()[:3]] if vp else [0.0, 0.0, 0.0]
            body = re.sub(r'\s*<!--.*?-->', '', body, flags=re.S)
            body = re.sub(r'\s*<pose>[^<]*</pose>', '', body, count=1)
            body = body.replace('meshes/', 'file://' + os.path.join(depot, 'meshes') + '/')
            body = body.replace('>materials/', '>file://' + os.path.join(depot, 'materials') + '/')
            body = re.sub(r'\s*<script>.*?</script>', '', body, flags=re.S)
            offset = tuple(a + b for a, b in zip(link_pose, vis_pose))
            visuals.setdefault(name, []).append((body, offset))
    return visuals


def visual_xml(name, body, pose, scale):
    body = body.replace('<scale>0.6 0.6 0.6</scale>', '<scale>%.4f %.4f %.4f</scale>' % scale)
    return ('        <visual name="%s">\n          <pose>%.3f %.3f %.3f 0 0 0</pose>%s\n        </visual>\n'
            % (name, pose[0], pose[1], pose[2], body.rstrip()))


def box_collision(name, pose, size):
    return ('        <collision name="%s">\n          <pose>%.3f %.3f %.3f %s</pose>\n'
            '          <geometry><box><size>%s</size></box></geometry>\n        </collision>\n'
            % (name, pose[0], pose[1], pose[2], ' '.join('%g' % a for a in pose[3:6]), ' '.join(size)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tiles-x', type=int, default=4)
    ap.add_argument('--tiles-y', type=int, default=4)
    ap.add_argument('--aisle', type=float, default=6.0, help='largura dos corredores entre blocos (m)')
    ap.add_argument('--height', type=float, default=12.0, help='altura do teto (m)')
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'worlds', 'big_depot.sdf'))
    a = ap.parse_args()

    depot = find_depot()
    visuals = read_visuals(depot)
    boxset_mesh = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(a.out)), 'meshes', 'depot_boxset.dae'))
    removed, total = make_boxset_mesh(depot, boxset_mesh)
    print('BOXSET: %d de %d triangulos tirados (pecas de parede)' % (removed, total))
    visuals['BOXSET_visual'] = [(body.replace('file://' + os.path.join(depot, 'meshes', 'Depot.dae'), 'file://' + boxset_mesh), off)
                                for body, off in visuals['BOXSET_visual']]
    collisions = find_collisions()

    dx = (CORE_MAX_X - CORE_MIN_X) + a.aisle
    dy = (CORE_MAX_Y - CORE_MIN_Y) + a.aisle
    margin = a.aisle + 2.0
    min_x, max_x = CORE_MIN_X - margin, CORE_MAX_X + (a.tiles_x - 1) * dx + margin
    min_y, max_y = CORE_MIN_Y - margin, CORE_MAX_Y + (a.tiles_y - 1) * dy + margin
    size_x, size_y = max_x - min_x, max_y - min_y
    cx, cy = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0
    kx, ky, kz = size_x / ORIG_X, size_y / ORIG_Y, a.height / ORIG_HEIGHT
    lift = a.height - ORIG_HEIGHT

    out = []
    # casca esticada
    shell_scale = (ORIG_SCALE * kx, ORIG_SCALE * ky, ORIG_SCALE * kz)
    shell_pose = (cx, cy - ORIG_Y_CENTER * ky, 0.0)
    for part in SHELL_PARTS:
        for body, _ in visuals.get(part, []):
            out.append(visual_xml(part, body, shell_pose, shell_scale))
    # miolo em grade
    n = 0
    for i in range(a.tiles_x):
        for j in range(a.tiles_y):
            ox, oy = i * dx, j * dy
            for part in CORE_PARTS + CEILING_PARTS:
                z = lift if part in CEILING_PARTS else 0.0
                for body, off in visuals.get(part, []):
                    n += 1
                    out.append(visual_xml('%s_%d_%d_%d' % (part, i, j, n), body,
                                          (ox + off[0], oy + off[1], z + off[2]),
                                          (ORIG_SCALE, ORIG_SCALE, ORIG_SCALE)))
            for name, pose, size in collisions:
                p = [pose[0] + ox, pose[1] + oy] + pose[2:]
                out.append(box_collision('%s_%d_%d' % (name, i, j), p, size))
    # paredes novas
    h = a.height
    walls = [((cx, min_y, h / 2), (size_x, 0.2, h)), ((cx, max_y, h / 2), (size_x, 0.2, h)),
             ((min_x, cy, h / 2), (0.2, size_y, h)), ((max_x, cy, h / 2), (0.2, size_y, h))]
    for k, (p, s) in enumerate(walls):
        out.append(box_collision('wall%d' % k, list(p) + [0, 0, 0], ['%.3f' % v for v in s]))

    # no corredor transversal entre a 1a e a 2a fileira, longe da parede oeste (a camera
    # que segue a empilhadeira fica 8 m atras)
    start = (CORE_MIN_X + 8.0, (CORE_MAX_Y + CORE_MIN_Y + dy) / 2.0)
    world = TEMPLATE.format(size_x=size_x, size_y=size_y, height=a.height, tiles_x=a.tiles_x,
                            tiles_y=a.tiles_y, aisle=a.aisle, start_x=start[0], start_y=start[1],
                            ground=max(size_x, size_y) * 3, body=''.join(out))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    world = portable(world, os.path.dirname(os.path.abspath(a.out)))
    for p in leftovers(world):
        print('AVISO: caminho absoluto no mundo:', p)
    open(a.out, 'w').write(world)
    print('%s: %.0f x %.0f m, %d x %d blocos, corredores de %.1f m, inicio sugerido (%.1f, %.1f)'
          % (os.path.abspath(a.out), size_x, size_y, a.tiles_x, a.tiles_y, a.aisle, start[0], start[1]))


TEMPLATE = '''<?xml version="1.0"?>
<!--
  Deposito grande para a simulacao da empilhadeira: {size_x:.0f} x {size_y:.0f} m, teto a {height:.0f} m,
  {tiles_x} x {tiles_y} blocos de prateleiras com corredores de {aisle:.1f} m.
  GERADO por tools/make_big_depot.py -- edite o gerador, nao este arquivo.
  Pecas do modelo "Depot" de OpenRobotics e colisoes do mundo "tugbot_depot" de MovAi, ambos do
  Gazebo Fuel, licenca CC-BY 4.0. A malha e as texturas sao lidas do cache ~/.gz/fuel.
  Inicio sugerido da empilhadeira: x {start_x:.1f}, y {start_y:.1f}, yaw 0.
-->
<sdf version="1.9">
  <world name="big_depot">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{ground:.0f} {ground:.0f}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{ground:.0f} {ground:.0f}</size></plane></geometry>
          <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="big_depot">
      <static>true</static>
      <link name="main">
{body}      </link>
    </model>
  </world>
</sdf>
'''

if __name__ == '__main__':
    main()
