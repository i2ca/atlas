#!/usr/bin/env python3
"""Achata as poses do SC-LIO-SAM (poses_opt.dat da VDU-02) para o chao plano: z = 0, roll = 0, pitch = 0; mantem x, y,
rumo e os tempos. Na simulacao o piso e' plano e o SC-LIO-SAM deriva em z/roll/pitch (no log de 23/10: z -4..+6 m,
roll ate' -20 graus, pitch ate' +19): o graphslam_publish passa a pose 6D para o mapper e os raios inclinados batem
no chao, que vira obstaculo no mapa 2D.

    flatten_poses.py <pasta sc_lio_sam do log>     (ex.: ../data/empilhadeira_sim/geodata/<log>/sc_lio_sam)

Guarda o original em poses_opt_original.dat (na primeira vez) e regrava poses_opt.dat achatado.
Formato (10 colunas): x y z roll pitch yaw t1 t2 t3 t4.
"""

import math
import os
import shutil
import sys


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    d = sys.argv[1]
    src = os.path.join(d, 'poses_opt.dat')
    orig = os.path.join(d, 'poses_opt_original.dat')
    if not os.path.exists(orig):
        shutil.copy2(src, orig)
    out, worst = [], [0.0, 0.0, 0.0]
    for line in open(orig):
        f = line.split()
        if len(f) != 10:
            continue
        worst = [max(worst[0], abs(float(f[2]))), max(worst[1], abs(float(f[3]))), max(worst[2], abs(float(f[4])))]
        f[2] = f[3] = f[4] = '0.000000'
        out.append(' '.join(f) + '\n')
    open(src, 'w').writelines(out)
    print('%s: %d poses achatadas (era: |z| ate %.2f m, |roll| ate %.1f graus, |pitch| ate %.1f graus); original em %s'
          % (src, len(out), worst[0], math.degrees(worst[1]), math.degrees(worst[2]), orig))


if __name__ == '__main__':
    main()
