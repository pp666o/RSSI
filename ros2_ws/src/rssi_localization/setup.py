from setuptools import find_packages, setup


package_name = "rssi_localization"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/localizer.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="dev@example.com",
    description="RSSI path localization package for ROS2 replay and online localization.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rssi_localizer_node = rssi_localization.rssi_localizer_node:main",
            "csv_replay_node = rssi_localization.csv_replay_node:main",
        ],
    },
)
