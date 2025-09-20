from setuptools import setup
import os

package_name = 'rbe550_grid_bench'

def data_files_list():
    files = [
        # Ament resource (critical)
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
    ]

    # Optional: maps (only if present in source tree)
    map_src = os.path.join(package_name, 'maps', 'maze_32.txt')  # src/rbe550_grid_bench/rbe550_grid_bench/maps/maze_32.txt
    if os.path.exists(map_src):
        files.append((os.path.join('share', package_name, 'maps'), [map_src]))

    # Optional: script
    script_src = os.path.join('scripts', 'run.sh')  # src/rbe550_grid_bench/scripts/run.sh
    if os.path.exists(script_src):
        files.append((os.path.join('share', package_name, 'scripts'), [script_src]))

    return files

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.algorithms'],
    package_dir={'': '.'},
    data_files=data_files_list(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='RBE-550 grid benchmark',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bench = rbe550_grid_bench.cli:main',
        ],
    },
)

