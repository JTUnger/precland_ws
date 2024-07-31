from setuptools import find_packages, setup
from glob import glob

package_name = 'caleuche_uav_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/calibration", glob("calibration/*.json")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joset',
    maintainer_email='josetomas.cl@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_camera_node = caleuche_uav_sensors.forward_camera_node:main',
            'downward_camera_node = caleuche_uav_sensors.downward_camera_node:main',
            'april_tag_detector = caleuche_uav_sensors.april_tag_detector:main',
            'precland_controller = caleuche_uav_sensors.precland_controller:main',
        ],
    },
)
