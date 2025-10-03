from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ms2r1p'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Herick Steven Duran Burgos',
    maintainer_email='tuemail@dominio.com',
    description='SCARA/MS2R1P manipulator package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dxf_parser_node = ms2r1p.dxf_parser_node:main',
            'trajectory_planner_node = ms2r1p.trajectory_planner_node:main',
            'trajectory_follower = ms2r1p.trajectory_follower:main',
            'inverse_kinematics = ms2r1p.inverse_kinematics:main',
            'direct_kinematics = ms2r1p.direct_kinematics:main',
            'ms2r1p_state_publisher = ms2r1p.ms2r1p_state_publisher:main',
        ],
    },
)
