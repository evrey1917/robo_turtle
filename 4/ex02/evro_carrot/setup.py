from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'evro_carrot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evrey',
    maintainer_email='nikitaevreev1917@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_broadcaster = evro_carrot.turtle_broadcaster:main',
            'turtle_listener = evro_carrot.turtle_listener:main',
            'dynamic_frame_broadcaster = evro_carrot.dynamic_frame_broadcaster:main',
        ],
    },
)
