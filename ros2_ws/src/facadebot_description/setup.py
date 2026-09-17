import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'facadebot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'meshes', 'v1'), glob('meshes/v1/*.STL')),
        (os.path.join('share', package_name, 'meshes', 'v2'), glob('meshes/v2/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harthik',
    maintainer_email='vaylarobotics@gmail.com',
    description='Arm URDFs, meshes, and the active-model config shared by the control stack',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
