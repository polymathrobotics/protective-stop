# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'protective_stop_remote'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['protective_stop_remote', 'protective_stop_remote.*']),
    install_requires=['RPi.GPIO', 'flask', 'roslibpy', 'smbus'],
    entry_points={
        'console_scripts': [
            'protective_stop_remote = protective_stop_remote.main:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/ament_index/resource_index/packages'), ['resource/' + package_name]),
        ('share/' + package_name + '/tests', glob('tests/*.py')),
        ('share/' + package_name + '/tests', glob('tests/pytest.ini')),
    ],
    zip_safe=True,
)
