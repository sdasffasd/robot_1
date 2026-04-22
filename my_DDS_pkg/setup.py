import os
import glob
from setuptools import setup

package_name = 'my_DDS_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='1461190907@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_001_speed = my_DDS_pkg.robot_001_speed:main',
            'robot_001_camera = my_DDS_pkg.robot_001_camera:main',
            'robot_001_state_pub = my_DDS_pkg.robot_001_state_pub:main',
            'robot_001_task = my_DDS_pkg.robot_001_task:main',
        ],
    },
)
