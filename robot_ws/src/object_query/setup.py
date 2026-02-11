from setuptools import find_packages, setup

package_name = 'object_query'

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
    maintainer='phudh',
    maintainer_email='dohuuphu25@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_query_server = object_query.object_query_server:main',
            'object_query_client = object_query.object_query_client:main',
            'coord_bridge = object_query.coord_bridge:main',
            'object_query_test = object_query.object_query_test:main',
        ],
    },
)
