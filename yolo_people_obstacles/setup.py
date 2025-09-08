from setuptools import setup
import os
from glob import glob

package_name = 'yolo_people_obstacles'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='YOLOv11 people detector → PointCloud2 for Nav2 costmap',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'yolo_people_to_cloud_node = yolo_people_obstacles.yolo_people_to_cloud_node:main',
        ],
    },
)
