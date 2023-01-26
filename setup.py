from setuptools import setup

package_name = 'turtlebot_aruco_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='t0nik',
    maintainer_email='antoni.gutowski@student.put.poznan.pl',
    description='TODO: Description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'detect_aruco = turtlebot_aruco_control.detect_aruco:main',
			'control_bot = turtlebot_aruco_control.control_turtlebot:main',
        ],
    },
)
