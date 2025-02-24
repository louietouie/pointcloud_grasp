from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pointcloud_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='louis',
    maintainer_email='lgparizeau@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pc_processor_node = pointcloud_package.pc_processor_node:main',
            'camera_calibrator_node = pointcloud_package.camera_calibrator_node:main'
        ],
    },
)