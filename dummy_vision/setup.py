from setuptools import find_packages, setup

package_name = 'dummy_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hata_ros',
    maintainer_email='hata_ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector_node = dummy_controller.aruco_detector_node:main',
            'aruco_tracker_node = dummy_controller.aruco_tracker_node:main',
            'depth_detector_node = dummy_controller.depth_detector_node:main'
        ],
    },
)
