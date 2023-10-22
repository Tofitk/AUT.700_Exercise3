from setuptools import find_packages, setup

package_name = 'robo_position_controller'

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
    maintainer='Topi Kärki',
    maintainer_email='topi.karki@tuni.fi',
    description='very simple robot controllers. Contains moving to point, point with specific angle, following 2 different predetermined paths',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'full_controller = robo_position_controller.robo_position_controller_node:main',
        'move_xy = robo_position_controller.move_to_xy:main',
        'move_xytheta = robo_position_controller.move_to_xytheta:main',
        'move_path = robo_position_controller.move_to_path:main'
        ],
    },
)
