from setuptools import find_packages, setup

package_name = 'facade_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harthik',
    maintainer_email='vaylarobotics@gmail.com',
    description='Cartesian move commands: inverse kinematics from a tool-tip target to joint angles',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'facade_control_node = facade_control.facade_control_node:main',
            'trajectory_node = facade_control.trajectory_node:main',
        ],
    },
)
