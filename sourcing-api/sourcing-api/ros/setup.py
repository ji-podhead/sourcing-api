import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ros_api_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('src/services/srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@example.com',
    description='API package for GigE camera driver.',
    license='TODO',
    tests_require=['ament_copyright', 'ament_flake8', 'ament_pep257', 'python3-pytest'],
    entry_points={
        'console_scripts': [
            # Example entry point, adjust if needed
            # 'gig_e_camera_node = api.src.devices.gig_e_driver:GigECameraNode',
        ],
    },
)
