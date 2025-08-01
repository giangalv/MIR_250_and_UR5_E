import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mir_manual_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gianluca Galvagni',
    maintainer_email='gianluca.galvagni@edu.unige.it',
    description='MIR 250 Manual Navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_mode = mir_manual_navigation.manual_mode:main',
            'cloud_transformation = mir_manual_navigation.cloud_transformation:main',
            'encoder_to_joint_state = mir_manual_navigation.encoder_to_joint_state:main',
            'initial_position = mir_manual_navigation.initial_position:main',
        ],
    },
)
