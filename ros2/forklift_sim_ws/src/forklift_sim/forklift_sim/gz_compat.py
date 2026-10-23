"""gz.transport / gz.msgs do Python em qualquer ROS 2 com o Gazebo novo (gz-sim, nao o Gazebo Classic).

Os modulos mudam de nome com a versao do Gazebo:
  Jazzy (Harmonic): gz.transport13 + gz.msgs10    Kilted (Ionic): gz.transport14 + gz.msgs11
  Lyrical (Jetty):  gz.transport   + gz.msgs (sem numero)
Usa o primeiro par que existir; transport e msgs tem que ser da mesma versao.

    from forklift_sim.gz_compat import Node, msg
    Pose_V = msg('pose_v', 'Pose_V')
"""

import importlib

PAIRS = (('gz.transport', 'gz.msgs'), ('gz.transport15', 'gz.msgs12'), ('gz.transport14', 'gz.msgs11'),
         ('gz.transport13', 'gz.msgs10'))

transport = None
MSGS = None
for _t, _m in PAIRS:
    try:
        transport = importlib.import_module(_t)
        importlib.import_module(_m + '.empty_pb2')
        MSGS = _m
        break
    except ImportError:
        transport = None
if transport is None:
    raise ImportError('gz.transport/gz.msgs do Python nao encontrados (%s): rode com o setup.bash do ROS 2 '
                      'que tem o Gazebo' % ', '.join(t for t, _ in PAIRS))

Node = transport.Node


def msg(module, cls):
    """classe de mensagem: msg('pose_v', 'Pose_V') -> gz.msgsNN.pose_v_pb2.Pose_V"""
    return getattr(importlib.import_module('%s.%s_pb2' % (MSGS, module)), cls)
