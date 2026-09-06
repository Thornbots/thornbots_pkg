# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'thornbots_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    # colcon only runs its pytest step when the package declares pytest
    # here; without it `colcon test` silently reports 0 tests and test/
    # never runs.
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='baptisbc@rose-hulman.edu',
    description='Sentry hardware interface and robot description for RHIT Thornbots ARC 2026',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_translator = thornbots_pkg.pose_translator:main',
            'odom_tf_broadcaster = thornbots_pkg.odom_tf_broadcaster:main',
            'lidar_self_filter = thornbots_pkg.lidar_self_filter:main',
            'mcb_relay = thornbots_pkg.mcb_relay:main',
            'point_to_cv_target = thornbots_pkg.point_to_cv_target:main',
            'target_selector = thornbots_pkg.target_selector:main',
            'target_tracker = thornbots_pkg.target_tracker:main',
        ],
    },
)
