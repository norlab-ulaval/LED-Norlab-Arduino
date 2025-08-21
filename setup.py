from setuptools import find_packages, setup

package_name = 'warthog_lights_arduino'

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
    maintainer='benoit',
    maintainer_email='renald.89@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_dummy = warthog_mcu_link.ros_dummy_publisher:main', 
            'warthog_lights_arduino = warthog_mcu_link.warthog_lights_arduino:main',
            'estop_state_publisher = warthog_mcu_link.estop_state_publisher:main',
        ],
    },
)
