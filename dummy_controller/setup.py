from setuptools import find_packages, setup

package_name = 'dummy_controller'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    #packages=[package_name, f'{package_name}.dummy_cli_tool'],
    packages=find_packages(include=[package_name, f'{package_name}.*']),
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
            'dummy_arm_controller = dummy_controller.dummy_arm_controller:main',
            'moveit_server = dummy_controller.moveit_server:main',
            'add_collision_object = dummy_controller.add_collision_object:main'
        ],
    },
)
