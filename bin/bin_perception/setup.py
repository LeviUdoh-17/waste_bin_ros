from setuptools import find_packages, setup

package_name = 'bin_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chibueze',
    maintainer_email='praiseorji4@gmail.com',
    description='YOLO-based object detection node for the bin robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_detector = bin_perception.yolo_detector:main',
            'scan_gate     = bin_perception.scan_gate:main',
        ],
    },
)
