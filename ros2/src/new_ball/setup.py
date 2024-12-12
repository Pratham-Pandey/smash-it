from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'new_ball'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratham',
    maintainer_email='prathampandey3172@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        # 'spawn = new_ball.random_spawn:main',
        'spawn = new_ball.random_spawn_v2:main',
        'talker = new_ball.calc_trajectory:main',            
        ],
    },
)
