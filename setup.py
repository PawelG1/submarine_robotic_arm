from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'sub_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    # URDF
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    # Launch files
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    # RViz config (we keep both rviz/ and config/ patterns; actual file is in config/)
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib', 'python-can'],
    zip_safe=True,
    maintainer='mecharolnik',
    maintainer_email='pawelgalka153@gmail.com',
    description='Submarine robotic arm tools, and visu',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_visu = sub_arm.my_visu:main',
            'target_publisher = sub_arm.target_publisher:main',
            'arm_state_publisher = sub_arm.arm_state_publisher:main',
            'ik_ui = sub_arm.ik_ui:main',
            'odrive_can = sub_arm.odrive_can:main',
        ],
    },
)
