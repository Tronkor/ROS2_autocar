from setuptools import find_packages, setup

package_name = 'python_pkg'

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
    maintainer='davinci-mini',
    maintainer_email='davinci-mini@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'lidar_sub = python_pkg.lidar_sub:main',
          'ackerman_pub = python_pkg.ackerman_pub:main', 
          'video = python_pkg.video:main',
          'perception_node  = python_pkg.perception_node:main',
          'control_node = python_pkg.control_node:main',
          'camlidar = python_pkg.camlidar:main',
          'perceptiondraw  = python_pkg.perceptiondraw:main',
          'file_reader_python = python_pkg.file_reader_python:main',
          'detection_processor = python_pkg.detection_processor:main',
        ],
    },
)
