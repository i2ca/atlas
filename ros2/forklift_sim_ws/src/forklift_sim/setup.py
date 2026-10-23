import os
from glob import glob
from setuptools import setup

package_name = 'forklift_sim'


def tree(src):
    """data_files de uma pasta com subpastas (worlds/models/<modelo>/...), mantendo a estrutura"""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join('share', package_name, root), [os.path.join(root, f) for f in files]))
    return out

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds', 'meshes'), glob('worlds/meshes/*')),
    ] + tree('worlds/models'),
    install_requires=['setuptools'],
    zip_safe=True,
    description='Simulador Gazebo da empilhadeira Hyster comandado pelo CAN',
    license='TODO',
    entry_points={
        'console_scripts': [
            'can_bridge = forklift_sim.can_bridge:main',
            'camera_follow = forklift_sim.camera_follow:main',
            'camera_stream = forklift_sim.camera_stream:main',
        ],
    },
)
