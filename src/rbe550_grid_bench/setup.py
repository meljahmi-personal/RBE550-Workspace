from setuptools import setup
import os
from glob import glob

package_name = 'rbe550_grid_bench'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.algorithms'],
    data_files=[
        # Register package with ament index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Eljahmi',
    maintainer_email='meeljahmi@wpi.edu',
    description='RBE-550 grid benchmark',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bench = rbe550_grid_bench.cli:main',
            'planner_node = rbe550_grid_bench.planner_node:main',
            'plot_bench = rbe550_grid_bench.plot_bench_all:main',
        ],
    },

)



