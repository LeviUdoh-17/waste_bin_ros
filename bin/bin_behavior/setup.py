from setuptools import setup

package_name = 'bin_behavior'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'color_detector   = bin_behavior.color_detector:main',
            'behavior_manager = bin_behavior.behavior_manager:main',
            'waypoint_runner  = bin_behavior.waypoint_runner:main',
            'waypoint_capture = bin_behavior.waypoint_capture:main',
        ],
    },
)
