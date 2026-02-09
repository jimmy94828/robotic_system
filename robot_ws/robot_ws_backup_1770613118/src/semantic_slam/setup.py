from setuptools import find_packages, setup

package_name = 'semantic_slam'

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
    description='Example SLAM Action server using semantic_slam_interfaces.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_slam_server = semantic_slam.run_slam_server:main',
            'run_slam_client = semantic_slam.run_slam_client:main',
        ],
    },
)
