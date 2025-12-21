from setuptools import setup
import os
from glob import glob

package_name = 'my_autonomous_robot'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name], yerine alttakini kullanmayı dene:
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdurrahman',
    maintainer_email='abdurrahmangulmez44@gmail.com',
    description='CEN449 Project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'explorer_node = my_autonomous_robot.explorer_node:main',
    ],
},
)