from setuptools import find_packages, setup

package_name = 'crazyflie_ros2_examples'

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
    maintainer='kimberly',
    maintainer_email='kimberly@bitcraze.io',
    description='Showing ROS 2 examples with the Crazyflie Library',
    license='MIT',
    tests_require=['pytest'],
entry_points={
        'console_scripts': [
                'talker = crazyflie_ros2_examples.cf_publisher:main',
        ],
},
)
