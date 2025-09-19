from setuptools import setup, find_packages

package_name = "rbe550_grid_bench"

setup(
    name=package_name,  # underscores only
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/bench.launch.py"]),
    ],
    install_requires=["setuptools", "ament_index_python", "numpy", "matplotlib", "pillow", "imageio"],
    zip_safe=True,
    maintainer="Mohamed Eljahmi",
    maintainer_email="you@example.com",
    description="RBE-550 gridworld benchmark",
    entry_points={
        "console_scripts": [
            "bench = rbe550_grid_bench.cli:main",
            "bench_runner = rbe550_grid_bench.bench_runner:main",
        ],
    },
)
