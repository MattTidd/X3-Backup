from setuptools import setup
import os
from glob import glob

package_name = 'x3_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
        (os.path.join('share','x3_description','rviz'),glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share','x3_description','meshes'),glob(os.path.join('meshes','*.STL'))),
        (os.path.join('share','x3_description','urdf'),glob(os.path.join('urdf','*.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matthew tidd',
    maintainer_email='mtidd2@unb.ca',
    description='allows for hardware communicate and generation of topics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'x3_driver = x3_bringup.x3_driver:main'
        ],
    },
)
