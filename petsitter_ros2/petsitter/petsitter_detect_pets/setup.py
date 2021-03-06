from setuptools import setup

package_name = 'petsitter_detect_pets'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='ivandiscobolo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'detect_pets_service=petsitter_detect_pets.detect_pets_service:main',
        	'detect_pets_service_real=petsitter_detect_pets.detect_pets_service_real:main'
        ],
    },
)
