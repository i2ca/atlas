#!/usr/bin/env python3
"""Gera o modelo 3D da Hyster para o viewer_3D do astro (OBJ + MTL), a partir do URDF do
Gazebo (Hyster_description), e os parametros carmodel_* para o astro.ini.

O viewer_3D (src/viewer_3D/draw_car.cpp) le' carmodel_file_name com glmReadOBJ, centraliza
(glmUnitize: a maior dimensao vira [-1, 1]), escala por carmodel_size_x/y/z / 2 e desenha
com: translate(carmodel_x, carmodel_y, carmodel_z + robot_wheel_radius), rotate +90 em x.
Ou seja, no OBJ: x = frente, y = cima, z = direita. Aqui o OBJ sai nesse referencial e os
carmodel_* saem calculados para o centro do modelo ficar no lugar certo em relacao ao
referencial do carro do astro -- que, na empilhadeira, e' o base_footprint (centro do eixo
dianteiro, no chao, x para o lado da garra).

As malhas STL tem ~300 mil triangulos (pneus com banda desenhada); o viewer_3D desenha o modelo
a cada quadro, entao cada peca e' simplificada por agrupamento de vertices numa grade.

Uso: make_viewer3d_model.py [--out <dir>] [--ini <astro.ini a atualizar>]
"""

import argparse
import math
import os
import re
import struct
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.abspath(os.path.join(HERE, '..', '..', 'Hyster_description'))

# grade da simplificacao (m): pneus mais grossos, resto mais fino
CELL_DEFAULT = 0.04
CELL_TIRES = 0.07


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(origin):
    t = np.eye(4)
    if origin is not None:
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        t[:3, :3] = rpy_matrix(*rpy)
        t[:3, 3] = xyz
    return t


def read_stl(path):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    raw = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]),
                        count=n, offset=84)
    return raw['v'].astype(np.float64)       # (n, 3, 3)


def simplify(tris, cell):
    """Agrupamento de vertices: cada vertice vai para o centro do seu cubo da grade."""
    pts = tris.reshape(-1, 3)
    keys = np.floor(pts / cell).astype(np.int64)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    sums = np.zeros((len(uniq), 3))
    np.add.at(sums, inverse, pts)
    counts = np.bincount(inverse, minlength=len(uniq))[:, None]
    verts = sums / counts
    faces = inverse.reshape(-1, 3)
    ok = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    faces = faces[ok]
    faces = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)[1]
    faces = inverse.reshape(-1, 3)[ok][np.sort(faces)]
    return verts, faces


def materials():
    """Cores do materials.xacro (rgba das propriedades)."""
    text = open(os.path.join(DESC, 'urdf', 'materials.xacro')).read()
    props = dict(re.findall(r'<xacro:property name="([^"]*)_rgba" value="([^"]*)"', text))
    colors = {name: [float(v) for v in rgba.split()[:3]] for name, rgba in props.items()}
    colors.setdefault('silver', [0.7, 0.7, 0.7])
    return colors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(os.environ.get('ASTRO_HOME', os.path.expanduser('~/astro')), 'data', 'empilhadeira_sim', 'models'))
    ap.add_argument('--ini', default=None, help='astro.ini para atualizar os carmodel_*')
    a = ap.parse_args()

    urdf = subprocess.run(['xacro', os.path.join(DESC, 'urdf', 'Hyster.xacro')], check=True,
                          capture_output=True, text=True).stdout
    root = ET.fromstring(urdf)
    joints = {j.find('child').get('link'): (j.find('parent').get('link'), transform(j.find('origin')))
              for j in root.findall('joint')}

    def link_pose(name):
        t = np.eye(4)
        while name in joints:
            parent, tj = joints[name]
            t = tj @ t
            name = parent
        return t

    colors = materials()
    os.makedirs(a.out, exist_ok=True)
    obj_path = os.path.join(a.out, 'hyster.obj')
    mtl_path = os.path.join(a.out, 'hyster.mtl')
    all_verts = []
    blocks = []
    offset = 1
    total_in = total_out = 0
    for link in root.findall('link'):
        vis = link.find('visual')
        if vis is None or vis.find('geometry/mesh') is None:
            continue
        mesh = vis.find('geometry/mesh')
        fname = re.sub(r'^.*/meshes/', '', mesh.get('filename'))
        scale = [float(v) for v in mesh.get('scale', '1 1 1').split()]
        tris = read_stl(os.path.join(DESC, 'meshes', fname)) * scale
        t = link_pose(link.get('name')) @ transform(vis.find('origin'))
        pts = tris.reshape(-1, 3) @ t[:3, :3].T + t[:3, 3]
        cell = CELL_TIRES if 'wheel' in link.get('name').lower() else CELL_DEFAULT
        verts, faces = simplify(pts.reshape(-1, 3, 3), cell)
        total_in += len(tris)
        total_out += len(faces)
        mat = vis.find('material')
        mname = mat.get('name') if mat is not None else 'silver'
        all_verts.append(verts)
        blocks.append((link.get('name'), mname, faces + offset))
        offset += len(verts)

    verts = np.vstack(all_verts)                       # referencial do base_footprint (x frente, y esq, z cima)
    mn, mx = verts.min(0), verts.max(0)
    center, size = (mn + mx) / 2.0, mx - mn

    with open(obj_path, 'w') as f:
        f.write('# Hyster para o viewer_3D -- GERADO por forklift_sim/tools/make_viewer3d_model.py\n')
        f.write('# eixos do viewer_3D: x = frente, y = cima, z = direita; metros\nmtllib hyster.mtl\n')
        # normais por vertice (media das faces, ponderada pela area) e uma coordenada de textura
        # unica: o draw_car chama o glmDraw com GLM_SMOOTH | GLM_TEXTURE, e sem vn/vt o GLM imprime
        # dois avisos a CADA quadro (saida que vai para o pipe do proccontrol)
        allf = np.vstack([fc - 1 for _, _, fc in blocks])
        fn = np.cross(verts[allf[:, 1]] - verts[allf[:, 0]], verts[allf[:, 2]] - verts[allf[:, 0]])
        vn = np.zeros_like(verts)
        for k in range(3):
            np.add.at(vn, allf[:, k], fn)
        vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)
        for v in verts:
            f.write('v %.4f %.4f %.4f\n' % (v[0], v[2], -v[1]))
        for n in vn:
            f.write('vn %.4f %.4f %.4f\n' % (n[0], n[2], -n[1]))
        f.write('vt 0 0\n')
        for name, mname, faces in blocks:
            f.write('g %s\nusemtl %s\n' % (name, mname))
            for tri in faces:
                f.write('f %d/1/%d %d/1/%d %d/1/%d\n' % (tri[0], tri[0], tri[1], tri[1], tri[2], tri[2]))
    with open(mtl_path, 'w') as f:
        for name in sorted({b[1] for b in blocks}):
            c = colors.get(name, colors['silver'])
            f.write('newmtl %s\nKa %.3f %.3f %.3f\nKd %.3f %.3f %.3f\nKs 0.2 0.2 0.2\nNs 20\nd 1\n\n'
                    % (name, c[0] * 0.4, c[1] * 0.4, c[2] * 0.4, c[0], c[1], c[2]))

    # glmUnitize: a maior dimensao vira [-1, 1]; com carmodel_size igual nos 3 eixos o modelo
    # volta ao tamanho real.
    biggest = float(size.max())
    print('%s: %d -> %d triangulos; tamanho %.2f x %.2f x %.2f m (comp. x larg. x alt.)'
          % (obj_path, total_in, total_out, size[0], size[1], size[2]))
    params = {
        'carmodel_file_name': '../data/empilhadeira_sim/models/hyster.obj',
        'carmodel_size_x': '%.3f' % biggest, 'carmodel_size_y': '%.3f' % biggest, 'carmodel_size_z': '%.3f' % biggest,
        'carmodel_x': '%.3f' % center[0], 'carmodel_y': '%.3f' % center[1],
        'carmodel_z_mais_raio_da_roda': '%.3f' % center[2],
    }
    if a.ini:
        text = open(a.ini).read()
        wheel = float(re.search(r'^robot_wheel_radius\s+([-0-9.]+)', text, re.M).group(1))
        params['carmodel_z'] = '%.3f' % (center[2] - wheel)
        del params['carmodel_z_mais_raio_da_roda']
        for k, v in params.items():
            pat = re.compile(r'^%s[ \t]+[^\n]*$' % k, re.M)
            line = '%s\t\t%s\t# EMPILHADEIRA: gerado por make_viewer3d_model.py' % (k, v)
            text = pat.sub(line, text, count=1) if pat.search(text) else text
        open(a.ini, 'w').write(text)
        print('atualizado:', a.ini)
    for k, v in params.items():
        print('  %s %s' % (k, v))


if __name__ == '__main__':
    main()
