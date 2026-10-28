"""URIs portaveis para os mundos gerados (make_big_depot.py, make_port_world.py).

Os geradores montam caminhos absolutos da maquina onde rodam (o cache do Gazebo Fuel em ~/.gz/fuel e a pasta
worlds/ deste pacote). Antes de gravar o .sdf, troca:
  - arquivos de worlds/ -> caminho RELATIVO ao .sdf (meshes/..., models/...): o Gazebo resolve pela pasta do mundo
    instalado (install/forklift_sim/share/forklift_sim/worlds);
  - arquivos do cache do Fuel -> URL do Fuel (https://fuel.gazebosim.org/1.0/<dono>/<models|worlds>/<nome>/<versao>/
    files/...): o Gazebo baixa sozinho para o cache na primeira vez (precisa de internet so' nessa vez).
Conferido no Gazebo 10.5 (23/10/2026): relativo e URL do Fuel resolvem em <uri> e em <albedo_map>.
"""

import os
import re

FUEL_CACHE = os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org')


def portable(text, worlds_dir):
    worlds_dir = os.path.abspath(worlds_dir).rstrip('/') + '/'
    fuel = re.escape(os.path.abspath(FUEL_CACHE)) + r'/([^/<"]+)/(models|worlds)/([^/<"]+)/(\d+)/'

    def fuel_url(m):
        return 'https://fuel.gazebosim.org/1.0/%s/%s/%s/%s/files/' % (m.group(1), m.group(2), m.group(3), m.group(4))
    # com e sem file:// (o <albedo_map> leva o caminho puro)
    text = re.sub(r'(?:file://)?' + fuel, fuel_url, text)
    text = re.sub(r'(?:file://)?' + re.escape(worlds_dir), '', text)
    return text


def leftovers(text):
    """caminhos absolutos que sobraram (para avisar)"""
    return sorted(set(re.findall(r'(?:file://)?/(?:home|root|Users)/[^<"\s]+', text)))
