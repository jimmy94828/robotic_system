from setuptools import find_packages, setup

package_name = 'map_alignment'

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
            'map_alignment = map_alignment.map_alignment_v2:main',
            'collect_data = map_alignment.collect_data:main',
            'lidar_camera_bridge = map_alignment.lidar_camera_link_bridge:main',
        ],
    },
)
