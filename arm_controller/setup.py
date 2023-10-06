from setuptools import find_packages, setup

package_name = 'arm_controller'

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
    maintainer='riley',
    maintainer_email='rellis5@hawk.iit.edu',
    description='Package contains nodes that control my robot arm manipulator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['servo_controller = arm_controller.servo_control_node:main',
                            'arduino_comms = arm_controller.arduino_communication:main'
        ],
    },
)
