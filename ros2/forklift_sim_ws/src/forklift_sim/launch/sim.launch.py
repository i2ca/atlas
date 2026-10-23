"""Hyster no Gazebo com ros2_control e a ponte CAN.

ros2 launch forklift_sim sim.launch.py [world:=depot] [gui:=false] [can:=false] [can_interface:=vcan0]
                                       [bridge_config:=<yaml>] [x:=.. y:=.. yaw:=..]

world: empty (padrao), depot (galpao do Gazebo Fuel, worlds/depot.sdf), big_depot, port (big_depot aberto
       para um patio de porto de celulose com cais e navio, tools/make_port_world.py) ou o caminho de um .sdf.
lidar: true sobe o LiDAR (gpu_lidar) montado e configurado pelo astro.ini (astro_ini:=, lidar_id:=):
       pose = sensor_board_1 * lidar<N>, grade vertical cobrindo lidar<N>_vertical_angles, tiros por
       volta = 1 / (lidar_rate * lidar<N>_time_between_shots) (20 Hz: 1800, como o OT-128 da IARA). O forklift_lidar_sim (astro) le' o mesmo .ini.
extra_lidars: lista de LiDARs alem do 0, ex. "1 2 3 4": cada um montado e configurado pelo seu
         lidarN_* do astro.ini (pose, vertical_angles, time_between_shots...), a extra_lidar_rate (10 Hz)
         e grade vertical extra_lidar_vstep_deg (0,5 grau), com a IMU interna em /lidarN/imu.
         Um forklift_lidar_sim -lidar_id N -rate <extra_lidar_rate> por LiDAR publica no astro.
cameras: true sobe as 3 cameras (1 frente, 2 re, 3 pinca) configuradas pelos intelbrasN_* do
         astro.ini, e um camera_stream por camera mandando H.264 para o intelbrasN_ip_adress, que
         o camera_drivers original abre.
camera: carry (padrao: a camera da janela vai junto com a empilhadeira sem girar nas curvas;
        zoom e rotacao com o mouse livres), free_look e follow (os modos do Gazebo: presos ao
        veiculo, giram junto e travam o zoom) ou free (solta).
"""

import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def read_ini(path):
    values = {}
    for line in open(path, errors='replace'):
        m = re.match(r'^([A-Za-z_][A-Za-z0-9_]*)\s+([^#\n]*)', line)
        if m and m.group(1) not in values:
            values[m.group(1)] = m.group(2).strip()
    return values


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def lidar_mappings(ini_path, lidar_id, vstep_deg, rate):
    """Pose e grade do gpu_lidar a partir do astro.ini (o mesmo que o param_daemon serve)."""
    v = read_ini(ini_path)
    lid = 'lidar%d_' % lidar_id

    def num(key, default=0.0):
        try:
            return float(v.get(key, default))
        except ValueError:
            return float(default)
    board_t = np.array([num('sensor_board_1_' + k) for k in ('x', 'y', 'z')])
    board_r = rotation(num('sensor_board_1_roll'), num('sensor_board_1_pitch'), num('sensor_board_1_yaw'))
    lidar_t = np.array([num(lid + k) for k in ('x', 'y', 'z')])
    lidar_r = rotation(num(lid + 'roll'), num(lid + 'pitch'), num(lid + 'yaw'))
    def compose(offset_t, offset_r):
        t = board_t + board_r @ offset_t
        r = board_r @ offset_r
        pitch = -math.asin(max(-1.0, min(1.0, r[2, 0])))
        return t, math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])
    t, roll, pitch, yaw = compose(lidar_t, lidar_r)
    # Xsens: xsens_x/y/z/roll/pitch/yaw em relacao a sensor_board_1 (0 se nao houver no .ini)
    xt, xroll, xpitch, xyaw = compose(np.array([num('xsens_' + k) for k in ('x', 'y', 'z')]),
                                      rotation(num('xsens_roll'), num('xsens_pitch'), num('xsens_yaw')))

    angles = [float(a) for a in v[lid + 'vertical_angles'].split()]
    # +-90 graus: o gpu_lidar nao passa do zenite (o JT128 vai ate' 90,5)
    vmin, vmax = max(min(angles) - 0.3, -90.0), min(max(angles) + 0.3, 90.0)
    vsamples = int(round((vmax - vmin) / vstep_deg)) + 1
    tbs = num(lid + 'time_between_shots')
    hsamples = int(round(1.0 / (rate * tbs))) if tbs > 0 else 1800
    if not 360 <= hsamples <= 10000:
        hsamples = 1800
    rdf = num(lid + 'range_division_factor', 1.0) or 1.0
    m = {'lidar': 'true',
         'lidar_pose': '%.4f %.4f %.4f %.5f %.5f %.5f' % (t[0], t[1], t[2], roll, pitch, yaw),
         'lidar_hsamples': str(hsamples), 'lidar_vsamples': str(vsamples),
         'lidar_vmin': '%.5f' % math.radians(vmin), 'lidar_vmax': '%.5f' % math.radians(vmax),
         'lidar_min_range': '%.3f' % (num(lid + 'min_sensing') / rdf), 'lidar_max_range': '%.1f' % num(lid + 'max_range', 100),
         'lidar_rate': '%g' % rate,
         'xsens_pose': '%.4f %.4f %.4f %.5f %.5f %.5f' % (xt[0], xt[1], xt[2], xroll, xpitch, xyaw)}
    print('[forklift_sim] %s (%s): pose %s | %d x %d raios a %g Hz | %s..%s m' % (
        lid[:-1], v.get(lid + 'model', '?'), m['lidar_pose'], hsamples, vsamples, rate, m['lidar_min_range'], m['lidar_max_range']))
    return m


def extra_lidars_sdf(ini_path, ids, vstep_deg, rate, urdf_xml):
    """<gazebo> com um gpu_lidar + imu por LiDAR extra, calculados como o lidar0 (lidar_mappings).
    lidarN_link (ex.: Base_das_garras_1) prende o sensor nesse link: a pose do .ini (garra em
    repouso) e' convertida para o referencial dele, e o sensor acompanha o link."""
    out = []
    v = read_ini(ini_path)
    for n in ids:
        m = lidar_mappings(ini_path, n, vstep_deg, rate)
        link = v.get('lidar%d_link' % n, 'base_footprint').split()[0] if v.get('lidar%d_link' % n) else 'base_footprint'
        if link != 'base_footprint':
            q = [float(x) for x in m['lidar_pose'].split()]
            in_car = np.eye(4)
            in_car[:3, :3] = rotation(*q[3:])
            in_car[:3, 3] = q[:3]
            pose = np.linalg.inv(link_rest_pose(urdf_xml, link)) @ in_car
            m['lidar_pose'] = '%.4f %.4f %.4f %.5f %.5f %.5f' % (tuple(pose[:3, 3]) + rpy_of(pose[:3, :3]))
            print('[forklift_sim] lidar%d preso em %s' % (n, link))
        out.append(f'''
  <gazebo reference="{link}">
    <sensor name="lidar{n}" type="gpu_lidar">
      <pose>{m['lidar_pose']}</pose>
      <topic>lidar{n}</topic>
      <update_rate>{m['lidar_rate']}</update_rate>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <lidar>
        <scan>
          <horizontal>
            <samples>{m['lidar_hsamples']}</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159265</min_angle>
            <max_angle>{3.14159265 - 2.0 * 3.14159265 / int(m['lidar_hsamples']):.8f}</max_angle>
          </horizontal>
          <vertical>
            <samples>{m['lidar_vsamples']}</samples>
            <resolution>1</resolution>
            <min_angle>{m['lidar_vmin']}</min_angle>
            <max_angle>{m['lidar_vmax']}</max_angle>
          </vertical>
        </scan>
        <range>
          <min>{m['lidar_min_range']}</min>
          <max>{m['lidar_max_range']}</max>
          <resolution>0.01</resolution>
        </range>
      </lidar>
    </sensor>
    <sensor name="lidar{n}_imu" type="imu">
      <pose>{m['lidar_pose']}</pose>
      <topic>lidar{n}/imu</topic>
      <update_rate>200</update_rate>
      <always_on>true</always_on>
    </sensor>
  </gazebo>''')
    return ''.join(out)


def link_rest_pose(urdf_xml, link):
    """Pose (4x4) de um link no referencial do base_footprint, com as juntas em zero."""
    root = ET.fromstring(urdf_xml)
    joints = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        t = np.eye(4)
        if o is not None:
            t[:3, :3] = rotation(*[float(v) for v in o.get('rpy', '0 0 0').split()])
            t[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
        joints[j.find('child').get('link')] = (j.find('parent').get('link'), t)
    t = np.eye(4)
    while link in joints:
        parent, tj = joints[link]
        t = tj @ t
        link = parent
    return t


def rpy_of(r):
    return (math.atan2(r[2, 1], r[2, 2]), -math.asin(max(-1.0, min(1.0, r[2, 0]))), math.atan2(r[1, 0], r[0, 0]))


def camera_mappings(ini_path, base_urdf_xml):
    """Poses/resolucao/campo de visao das 3 cameras (intelbrasN_* do astro.ini, em relacao a
    sensor_board_1) e os destinos dos streams."""
    v = read_ini(ini_path)

    def num(key, default=0.0):
        try:
            return float(v.get(key, default))
        except ValueError:
            return float(default)
    board = np.eye(4)
    board[:3, :3] = rotation(num('sensor_board_1_roll'), num('sensor_board_1_pitch'), num('sensor_board_1_yaw'))
    board[:3, 3] = [num('sensor_board_1_' + k) for k in ('x', 'y', 'z')]
    m = {'cameras': 'true'}
    streams = []
    width = int(num('intelbras1_width_1', 640))
    fx = num('intelbras1_fx', 381.4)
    m['cam_width'] = str(width)
    m['cam_height'] = str(int(num('intelbras1_height_1', 480)))
    m['cam_hfov'] = '%.5f' % (2.0 * math.atan(width / 2.0 / fx))
    m['cam_rate'] = '%g' % num('intelbras1_sim_rate', 15)
    for n in (1, 2, 3):
        c = 'intelbras%d_' % n
        cam = np.eye(4)
        cam[:3, :3] = rotation(num(c + 'roll'), num(c + 'pitch'), num(c + 'yaw'))
        cam[:3, 3] = [num(c + k) for k in ('x', 'y', 'z')]
        in_car = board @ cam
        link = v.get(c + 'link', 'base_footprint') or 'base_footprint'
        pose = in_car if link == 'base_footprint' else np.linalg.inv(link_rest_pose(base_urdf_xml, link)) @ in_car
        m['cam%d_link' % n] = link
        m['cam%d_pose' % n] = '%.4f %.4f %.4f %.5f %.5f %.5f' % (tuple(pose[:3, 3]) + rpy_of(pose[:3, :3]))
        url = v.get(c + 'ip_adress', 'udp://127.0.0.1:%d' % (5600 + n))
        streams.append((n, url))
        print('[forklift_sim] camera %d em %s: carro %.2f %.2f %.2f -> %s' % (
            n, link, in_car[0, 3], in_car[1, 3], in_car[2, 3], url))
    return m, streams


def world_for_launch(world_file):
    """Os mundos usam caminhos relativos a pasta worlds/ (e URLs do Fuel). O Gazebo trata URI nao resolvido como
    erro FATAL (o mundo inteiro nao carrega). Se faltar algum arquivo opcional -- o navio "Ship AAA" nao vai no
    repositorio (licenca), ver worlds/models/LEIA-ME.txt --, tira so' os visuais/colisoes que usam ele e sobe uma
    copia em /tmp com os caminhos absolutos desta instalacao. Sem nada faltando, usa o arquivo como esta'."""
    text = open(world_file).read()
    base = os.path.dirname(os.path.abspath(world_file))
    tags = r'(uri|albedo_map|normal_map|roughness_map|metalness_map|emissive_map)'

    def relative(u):
        return not re.match(r'^[a-z]+://', u) and not u.startswith('/')
    refs = set(m.group(2) for m in re.finditer(r'<%s>\s*([^<\s]+)\s*</\1>' % tags, text))
    missing = sorted(u for u in refs if relative(u) and not os.path.exists(os.path.join(base, u)))
    if not missing:
        return world_file
    def drop(m):
        return '' if any('>%s<' % u in m.group(0) for u in missing) else m.group(0)
    text = re.sub(r'<(visual|collision) name="[^"]*">.*?</\1>', drop, text, flags=re.S)
    text = re.sub(r'<%s>\s*([^<\s]+)\s*</\1>' % tags,
                  lambda m: '<%s>%s</%s>' % (m.group(1), ('file://' if m.group(1) == 'uri' else '') +
                                             os.path.join(base, m.group(2)) if relative(m.group(2)) else m.group(2),
                                             m.group(1)), text)
    out = os.path.join(tempfile.gettempdir(), 'forklift_sim_%s_%d.sdf' % (os.path.basename(world_file)[:-4], os.getpid()))
    open(out, 'w').write(text)
    print('[forklift_sim] AVISO: faltam no mundo (tirados): %s -- ver worlds/models/LEIA-ME.txt' % ', '.join(missing))
    return out


def grab_joints_sdf(world_file):
    """Uma DetachableJoint por unidade de celulose com fisica do mundo (modelos pulp_dyn_<n>, gerados por
    tools/make_port_world.py): solta no inicio, presa no braco Garra_1_1 quando o can_bridge publica em
    /hyster/grab/<nome>/attach (garra fechada na unidade) e solta em .../detach."""
    try:
        names = re.findall(r'<model name="(pulp_dyn_\d+)">', open(world_file).read())
    except OSError:
        return ''
    return ''.join(
        '<gazebo><plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">'
        '<parent_link>Garra_1_1</parent_link><child_model>%s</child_model><child_link>link</child_link>'
        '<initial_attach>false</initial_attach><suppress_child_warning>true</suppress_child_warning>'
        '<attach_topic>/hyster/grab/%s/attach</attach_topic><detach_topic>/hyster/grab/%s/detach</detach_topic>'
        '<output_topic>/hyster/grab/%s/state</output_topic></plugin></gazebo>\n' % (n, n, n, n) for n in names)


def generate_launch_description():
    share = get_package_share_directory('forklift_sim')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    controllers_yaml = os.path.join(share, 'config', 'controllers.yaml')

    # Onde a empilhadeira nasce em cada mundo (x, y, yaw), se x/y/yaw nao forem passados.
    # depot: area livre da ponta oeste. big_depot: corredor transversal, longe da parede.
    world_start = {'empty': ('0', '0', '0'), 'depot': ('-3.8', '1.1', '0'), 'big_depot': ('0.2', '8.8', '0'),
                   'port': ('5.0', '-32.0', '0')}

    def gazebo_and_spawn(context):
        mappings = {}
        if LaunchConfiguration('lidar').perform(context) == 'true':
            mappings = lidar_mappings(LaunchConfiguration('astro_ini').perform(context),
                                      int(LaunchConfiguration('lidar_id').perform(context)),
                                      float(LaunchConfiguration('lidar_vstep_deg').perform(context)),
                                      float(LaunchConfiguration('lidar_rate').perform(context)))
        streams = []
        if LaunchConfiguration('cameras').perform(context) == 'true':
            base_urdf = xacro.process_file(os.path.join(share, 'urdf', 'forklift.urdf.xacro'), mappings=mappings).toxml()
            cam_m, streams = camera_mappings(LaunchConfiguration('astro_ini').perform(context), base_urdf)
            mappings.update(cam_m)
        robot_description = xacro.process_file(os.path.join(share, 'urdf', 'forklift.urdf.xacro'),
                                               mappings=mappings).toxml()
        extra = [int(n) for n in LaunchConfiguration('extra_lidars').perform(context).replace(',', ' ').split()]
        if extra:
            robot_description = robot_description.replace('</robot>', extra_lidars_sdf(
                LaunchConfiguration('astro_ini').perform(context), extra,
                float(LaunchConfiguration('extra_lidar_vstep_deg').perform(context)),
                float(LaunchConfiguration('extra_lidar_rate').perform(context)), robot_description) + '\n</robot>')
        world = LaunchConfiguration('world').perform(context)
        name = world
        if os.path.isfile(os.path.join(share, 'worlds', world + '.sdf')):
            world_file = os.path.join(share, 'worlds', world + '.sdf')
        else:
            world_file = world
            name = ''
        grab = grab_joints_sdf(world_file)
        world_file = world_for_launch(world_file)
        if grab:
            robot_description = robot_description.replace('</robot>', grab + '</robot>')
        robot_state_publisher = Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

        # headless: o servidor precisa renderizar sozinho (gpu_lidar)
        server_only = '' if LaunchConfiguration('gui').perform(context) == 'true' else '-s --headless-rendering '
        start = world_start.get(name, ('0', '0', '0'))
        x = LaunchConfiguration('x').perform(context) or start[0]
        y = LaunchConfiguration('y').perform(context) or start[1]
        yaw = LaunchConfiguration('yaw').perform(context) or start[2]

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': f'-r {server_only}-v 3 {world_file}', 'on_exit_shutdown': 'true'}.items())

        # base_footprint fica no chao, no centro do eixo dianteiro
        spawn = Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-topic', '/robot_description', '-name', 'hyster', '-x', x, '-y', y, '-z', '0.01', '-Y', yaw])

        on_spawn = [spawner('joint_state_broadcaster'), spawner('mast_controller'),
                    spawner('steer_controller'), spawner('drive_controller')]
        camera = LaunchConfiguration('camera').perform(context)
        if server_only == '' and camera == 'carry':
            # camera_follow: a camera e' so' carregada junto (sem girar nas curvas); zoom e
            # rotacao do mouse continuam livres.
            on_spawn.append(Node(package='forklift_sim', executable='camera_follow', output='screen',
                                 arguments=['--model', 'hyster']))
        elif server_only == '' and camera in ('free_look', 'follow'):
            # Camera da janela do Gazebo presa na empilhadeira: 8 m atras e 3,8 m acima (abaixo
            # das vigas das prateleiras), rigida (pgain 1). free_look: vai junto, mas o mouse gira
            # a visao; follow: sempre olhando por tras. Repete porque a janela pode abrir depois.
            mode = 'FOLLOW_FREE_LOOK' if camera == 'free_look' else 'FOLLOW'
            track = (f'track_mode: {mode}, follow_target: {{name: "hyster"}}, '
                     'follow_offset: {x: -8, y: 0, z: 3.8}, follow_pgain: 1.0')
            on_spawn.append(ExecuteProcess(
                cmd=['bash', '-c', 'for i in 1 2 3 4 5 6; do sleep 2; '
                     f"gz topic -t /gui/track -m gz.msgs.CameraTrack -p '{track}'; done"],
                output='log'))
        controllers = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=on_spawn))
        # as "cameras IP": imagem do Gazebo -> H.264 em UDP para o camera_drivers
        camera_streams = [Node(package='forklift_sim', executable='camera_stream', name='camera_stream%d' % n,
                               output='screen', arguments=['--topic', 'forklift_cam%d/image' % n, '--url', url,
                                                           '--rate', mappings.get('cam_rate', '15')])
                          for n, url in streams]
        return [gazebo, robot_state_publisher, spawn, controllers] + camera_streams

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/model/hyster/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager-timeout', '60', '--param-file', controllers_yaml])

    can_bridge = Node(
        package='forklift_sim', executable='can_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('can')),
        parameters=[LaunchConfiguration('bridge_config'),
                    {'can_interface': LaunchConfiguration('can_interface')}])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('camera', default_value='carry'),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('cameras', default_value='false'),
        DeclareLaunchArgument('extra_lidars', default_value=''),
        DeclareLaunchArgument('extra_lidar_rate', default_value='10'),
        DeclareLaunchArgument('extra_lidar_vstep_deg', default_value='0.5'),
        DeclareLaunchArgument('astro_ini', default_value=os.path.join(
            os.environ.get('ASTRO_HOME', os.path.expanduser('~/astro')), 'data', 'empilhadeira_sim', 'parameters_ini_new', 'astro.ini')),
        DeclareLaunchArgument('lidar_id', default_value='0'),
        DeclareLaunchArgument('lidar_vstep_deg', default_value='0.125'),
        DeclareLaunchArgument('lidar_rate', default_value='20'),
        DeclareLaunchArgument('x', default_value=''),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        DeclareLaunchArgument('can', default_value='true'),
        DeclareLaunchArgument('can_interface', default_value='vcan0'),
        DeclareLaunchArgument('bridge_config', default_value=os.path.join(share, 'config', 'can_bridge.yaml')),
        OpaqueFunction(function=gazebo_and_spawn), clock_bridge, can_bridge,
    ])
