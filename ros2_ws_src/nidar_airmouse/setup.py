from setuptools import find_packages, setup

package_name = 'nidar_airmouse'

setup(
    name=package_name,
    version='0.0.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhishek',
    maintainer_email='abhishek@example.com',
    description='NIDAR AirMouse controller and PX4 simulation bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry = nidar_airmouse.telemetry:main',
            'airmouse = nidar_airmouse.airmouse_controller:main',
            'nidar_bridge = nidar_airmouse.nidar_bridge:main',
	    'px4_airmouse_sim = nidar_airmouse.px4_airmouse_sim:main',	
            'px4_setpoint_sim = nidar_airmouse.px4_setpoint_sim:main',
            'px4_offboard_safety = nidar_airmouse.px4_offboard_safety:main',
        ],
    },
)
