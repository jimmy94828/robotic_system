from setuptools import find_packages, setup

package_name = 'kachaka_laser_api'

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
    maintainer='acm118',
    maintainer_email='acm118@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'laser_from_api = kachaka_laser_api.kachaka_laser_from_api_node:main',
            'laser_to_ptcloud = kachaka_laser_api.scan2ptcloud:main',
        ],
    },
)
