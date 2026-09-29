import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'harvest_teleop'

def package_hardware_files():
    data_files = []
    for root, dirs, files in os.walk('hardware'):
        if '__pycache__' in root:
            continue
        
        file_paths = []
        for file in files:
            file_paths.append(os.path.join(root, file))
            
        if file_paths:
            install_dir = os.path.join('share', package_name, root)
            data_files.append((install_dir, file_paths))
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ] + package_hardware_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ari Hartawan',
    description='HART Framework: Hybrid Admittance Robotic Teleoperation for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = harvest_teleop.teleop_node:main',
            'keyboard_node = harvest_teleop.keyboard_node:main',
            'harvest_gui_logger = harvest_teleop.harvest_gui_logger:main',
            'tri_camera_node = harvest_teleop.tri_camera_node:main',
            'run_inference_safe = harvest_teleop.run_inference_safe:main',
            'harvest_webcam_gui_logger = harvest_teleop.harvest_webcam_gui_logger:main',
            'harvest_webcam_cartesian_gui_logger = harvest_teleop.harvest_webcam_cartesian_gui_logger:main',
            'harvest_lerobot_logger = harvest_teleop.harvest_lerobot_logger:main',
        ],
    },
)