import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sentry_pkg'

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
        (os.path.join('share', package_name, 'world'), glob('world/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='baptisbc@rose-hulman.edu',
    description='Sentry hardware interface and robot description for RHIT Thornbots ARC 2026',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_translator = sentry_pkg.pose_translator:main',
            'odom_tf_broadcaster = sentry_pkg.odom_tf_broadcaster:main',
            'lidar_self_filter = sentry_pkg.lidar_self_filter:main',
            'mcb_relay = sentry_pkg.mcb_relay:main',
            'point_to_cv_target = sentry_pkg.point_to_cv_target:main',
            'target_selector = sentry_pkg.target_selector:main',
        ],
    },
)
