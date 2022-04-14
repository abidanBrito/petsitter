import os
from glob import glob
from setuptools import setup


package_name = 'petsitter_nav2_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.pgm')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.model'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abidán Brito',
    maintainer_email='abidan.brito@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = petsitter_nav2_system.initial_pose_pub:main',
            'nav_to_pose = petsitter_nav2_system.nav_to_pose:main',
        ],
    },
)
