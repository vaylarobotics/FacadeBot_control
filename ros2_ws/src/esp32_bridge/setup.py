from setuptools import find_packages, setup

package_name = 'esp32_bridge'

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
    description='Bridges a joint-position command topic to the ESP32 over Wi-Fi/TCP',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'esp32_bridge_node = esp32_bridge.esp32_bridge_node:main'
        ],
    },
)
