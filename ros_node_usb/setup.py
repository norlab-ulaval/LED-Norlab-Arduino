from setuptools import find_packages, setup

package_name = 'usb_comms_test'

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
            'usb_comms_pub = usb_comms_test.usb_comms_pub:main', 
            'usb_color = usb_comms_test.usb_pub_test:main', 
        ],
    },
)
