from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'monk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf.xacro'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'source'), glob(os.path.join('source', '*.png'))),
        (os.path.join('share', package_name, 'source'), glob(os.path.join('source', '*.dae'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evrey',
    maintainer_email='nikitaevreev1917@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_movement = monk.circle_movement:main',
            'star_movement = monk.star_movement:main',
        ],
    },
)
