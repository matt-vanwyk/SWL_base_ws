from setuptools import find_packages, setup

package_name = 'base_package'

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
    maintainer='spiderweb',
    maintainer_email='matt.vanwyk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api_receive = base_package.api_receive:main',
            'api_post = base_package.api_post:main',
            'base_state_machine = base_package.base_state_machine:main',
            'arduino_node = base_package.arduino_node:main',
            'connection_monitor = base_package.connection_monitor:main'
        ],
    },
)
