from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # Keep this for the package itself
    #py_modules=['f1tenth_bringup.scripts.follow_these_waypoints'],  
    py_modules=['f1tenth_bringup.scripts.follow_these_waypoints_repeatedly'],  # Make sure Python sees the script module
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BP',
    maintainer_email='yoodyui@yahoo.com',
    description='Bringup for F1Tenth with Joy, VESC, and Ackermann nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'follow_these_waypoints = f1tenth_bringup.scripts.follow_these_waypoints:main',
            'follow_these_waypoints_repeatedly = f1tenth_bringup.scripts.follow_these_waypoints_repeatedly:main',
        ],
    },
)
