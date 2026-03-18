from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'sk_mpc_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch 文件（如果有）
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'cvxpy',
    ],
    zip_safe=True,
    author='Kewei ZUO',
    author_email='zuo-kewei@g.ecc.u-tokyo.ac.jp',
    maintainer='Kewei ZUO',
    maintainer_email='zuo-kewei@g.ecc.u-tokyo.ac.jp',
    description='Stability-index-based MPC controller and bridges for robotic tissue manipulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ===== MPC 本体 =====
            'mpc_node = sk_mpc_controller.mpc_node:main',

            # ===== perception -> MPC 输入桥接 =====
            'perception_to_mpc_inputs = sk_mpc_controller.perception_to_mpc_inputs_node:main',

            # ===== MPC -> robot command 桥接 =====
            'mpc_to_robot_command_bridge = sk_mpc_controller.mpc_to_robot_command_bridge:main',
            
            # ===== MPC 本体 2d =====
            'mpc_node_2d = sk_mpc_controller.mpc_node_2d:main',

            # ===== perception -> MPC 输入桥接  2d=====
            'perception_to_mpc_inputs_2d = sk_mpc_controller.perception_to_mpc_inputs_node_v2:main',

            # ===== MPC -> robot command 桥接 3d=====
            'mpc_to_robot_command_bridge_2d = sk_mpc_controller.dxy_to_robot_command_bridge:main',
            
            # ===== MPC -> topic 信息记录=====
            'csv_topic_logger = sk_mpc_controller.csv_topic_logger_node:main',
        ],
    },
)

