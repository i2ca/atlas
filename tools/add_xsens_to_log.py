#!/usr/bin/env python3
"""Recria o XSENS_QUAT de um log do simulador a partir do GPS (NMEAGGA1) e do HDT gravados, que no
simulador sao a pose verdadeira. Para log gravado com o forklift_gz_pose sem IMU (22/10: o log
log_emp-2026-10-22_20-47-07 saiu sem nenhum XSENS_QUAT e o SC-LIO-SAM depende dele).

    add_xsens_to_log.py <log.txt> [saida.txt] [astro.ini]

- saida padrao: <log>_xsens.txt, ao lado; o original nao e' tocado. As pastas de dados (<log>.txt_lidarN,
  _cameraN) entram como links simbolicos com o nome novo (o log as referencia pelo prefixo do nome).
- carro = antena - R(yaw) * (sensor_board_1 + gps_nmea_1); xsens = carro + R(yaw) * (sensor_board_1 + xsens)
- aceleracao: 2a derivada da posicao do xsens (20 Hz, suavizada) + gravidade, no referencial do sensor;
  giro: derivada do yaw; orientacao: so' yaw (chao plano). 100 Hz, interpolado.
- se o log ja' tiver XSENS_QUAT, nao faz nada.
"""

import math
import os
import re
import sys

import numpy as np

ASTRO_HOME = os.environ.get('ASTRO_HOME', os.path.expanduser('~/astro'))
DEFAULT_INI = os.path.join(ASTRO_HOME, 'data', 'empilhadeira_sim', 'parameters_ini_new', 'astro.ini')
RATE = 100.0
GRAVITY = 9.8


def utm(lat, lon, zone=24):
    a = 6378137.0; f = 1 / 298.257223563; k0 = 0.9996; e2 = f * (2 - f); ep2 = e2 / (1 - e2)
    lat = math.radians(lat); lon0 = math.radians((zone - 1) * 6 - 180 + 3); lon = math.radians(lon)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2); T = math.tan(lat) ** 2; C = ep2 * math.cos(lat) ** 2
    A = math.cos(lat) * (lon - lon0)
    M = a * ((1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * lat - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * lat)
             + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * lat) - (35 * e2 ** 3 / 3072) * math.sin(6 * lat))
    E = k0 * N * (A + (1 - T + C) * A ** 3 / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * A ** 5 / 120) + 500000
    Nn = k0 * (M + N * math.tan(lat) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A ** 4 / 24
                                        + (61 - 58 * T + T * T + 600 * C - 330 * ep2) * A ** 6 / 720))
    if lat < 0:
        Nn += 10000000
    return E, Nn


def read_ini(path):
    v = {}
    for line in open(path, errors='replace'):
        m = re.match(r'^([A-Za-z_]\w*)[ \t]+([^#\n]*)', line)
        if m and m.group(1) not in v:
            v[m.group(1)] = m.group(2).strip()
    return v


def smooth(x, n=5):
    k = np.ones(n) / n
    pad = np.pad(x, (n // 2, n // 2), mode='edge')
    return np.convolve(pad, k, mode='valid')


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src = os.path.abspath(sys.argv[1])
    dst = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else src[:-4] + '_xsens.txt'
    v = read_ini(sys.argv[3] if len(sys.argv) > 3 else DEFAULT_INI)
    f = lambda k: float(v.get(k, '0').split()[0])
    board = np.array([f('sensor_board_1_x'), f('sensor_board_1_y'), f('sensor_board_1_z')])
    gps = board + np.array([f('gps_nmea_1_x'), f('gps_nmea_1_y'), f('gps_nmea_1_z')])
    xs = board + np.array([f('xsens_x'), f('xsens_y'), f('xsens_z')])

    gga, hdt, ofs = [], [], []
    for line in open(src, errors='replace'):
        if line.startswith('XSENS_QUAT'):
            sys.exit('o log ja tem XSENS_QUAT: nada a fazer')
        if line.startswith('NMEAGGA1 '):
            p = line.split()
            la = int(float(p[3]) / 100) + (float(p[3]) % 100) / 60
            lo = int(float(p[5]) / 100) + (float(p[5]) % 100) / 60
            la, lo = (-la if p[4] == 'S' else la), (-lo if p[6] == 'W' else lo)
            E, N = utm(la, lo)
            gga.append((float(p[-3]), N, -E))
            ofs.append(float(p[-3]) - float(p[-1]))
        elif line.startswith('NMEAHDT '):
            p = line.split()
            hdt.append((float(p[-3]), float(p[2])))
    if len(gga) < 50 or len(hdt) < 50:
        sys.exit('log sem GPS/HDT suficiente (%d GGA, %d HDT)' % (len(gga), len(hdt)))
    gga, hdt = np.array(gga), np.array(hdt)
    offset = float(np.median(ofs))             # ipc_timestamp - logger_timestamp

    t = gga[:, 0]
    yaw = np.unwrap(np.interp(t, hdt[:, 0], np.unwrap(hdt[:, 1])))
    c, s = np.cos(yaw), np.sin(yaw)
    car_x = gga[:, 1] - (c * gps[0] - s * gps[1])
    car_y = gga[:, 2] - (s * gps[0] + c * gps[1])
    px = car_x + c * xs[0] - s * xs[1]
    py = car_y + s * xs[0] + c * xs[1]

    # grade uniforma a 20 Hz, derivadas e suavizacao
    tg = np.arange(t[0], t[-1], 0.05)
    gx, gy, gyaw = np.interp(tg, t, px), np.interp(tg, t, py), np.interp(tg, t, yaw)
    vx, vy = smooth(np.gradient(gx, 0.05)), smooth(np.gradient(gy, 0.05))
    ax, ay = smooth(np.gradient(vx, 0.05)), smooth(np.gradient(vy, 0.05))
    wz = smooth(np.gradient(gyaw, 0.05))

    to = np.arange(tg[0] + 0.1, tg[-1] - 0.1, 1.0 / RATE)
    oyaw = np.interp(to, tg, gyaw)
    oax, oay, owz = np.interp(to, tg, ax), np.interp(to, tg, ay), np.interp(to, tg, wz)
    cy, sy = np.cos(oyaw), np.sin(oyaw)
    fx = cy * oax + sy * oay                   # R(yaw)^T * (a + g z)
    fy = -sy * oax + cy * oay
    out = []
    for i, ts in enumerate(to):
        qw, qz = math.cos(oyaw[i] / 2), math.sin(oyaw[i] / 2)
        out.append((ts - offset, 'XSENS_QUAT %.6f %.6f %.6f %.6f 0.000000 0.000000 %.6f 0.200000 0.000000 -0.400000 '
                                 '0.000000 0.000000 %.6f 35.000000 %d %.6f add_xsens_to_log@%s %.6f\n'
                    % (fx[i], fy[i], GRAVITY, qw, qz, owz[i], i % 65536, ts, os.uname().nodename, ts - offset)))

    # junta no lugar certo (ordem do logger_timestamp, ultima coluna)
    k = 0
    with open(src, errors='replace') as fin, open(dst, 'w') as fout:
        for line in fin:
            if line.startswith('#') or line.startswith('PARAM') or not line.strip():
                fout.write(line)
                continue
            try:
                lt = float(line.split()[-1])
            except (ValueError, IndexError):
                fout.write(line)
                continue
            while k < len(out) and out[k][0] <= lt:
                fout.write(out[k][1])
                k += 1
            fout.write(line)
        while k < len(out):
            fout.write(out[k][1])
            k += 1

    links = []
    for d in sorted(os.listdir(os.path.dirname(src))):
        full = os.path.join(os.path.dirname(src), d)
        base = os.path.basename(src)
        if d.startswith(base + '_') and os.path.isdir(full):
            link = dst + d[len(base):]
            if not os.path.exists(link):
                os.symlink(full, link)
            links.append(os.path.basename(link))
    print('%s: %d XSENS_QUAT a %.0f Hz (%.0f s) | pastas ligadas: %s' % (dst, len(out), RATE, to[-1] - to[0],
                                                                         ' '.join(links)))
    print('aceleracao lateral max %.2f m/s^2 | giro max %.2f rad/s' % (np.abs(fy).max(), np.abs(owz).max()))


if __name__ == '__main__':
    main()
