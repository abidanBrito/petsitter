from setuptools import setup
import os
from glob import glob

package_name = 'petsitter_slam_auto'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
    description='Autonomous SLAM packagee',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
      	    'slam_server = petsitter_slam_auto.slam_server:main',
            'upload_map_firestore = petsitter_slam_auto.upload_map_firestore:main' 
        ],
    },
)
