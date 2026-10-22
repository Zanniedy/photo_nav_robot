from glob import glob
from setuptools import setup

package_name = 'person_locator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'calibrate = person_locator.calibrate:main',
            'camera_node = person_locator.camera_node:main',
            'calib_node = person_locator.calib_node:main',
            'person_tracker = person_locator.person_tracker_node:main',
            'approach_person = person_locator.approach_client:main',
        ],
    },
)
