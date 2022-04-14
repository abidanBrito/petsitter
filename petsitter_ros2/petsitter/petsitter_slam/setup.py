from setuptools import setup
import os #incluir
from glob import glob #incluir

package_name = 'petsitter_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'map'), glob('map/*.pgm')),#incluir
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml')),#incluir
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),#incluir
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),#incluir     
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),#incluir  
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abidán Brito, Iván Villanueva',
    maintainer_email='abricla@epsg.upv.es',
    description='ROS2 SLAM package for Gazebo test world.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
