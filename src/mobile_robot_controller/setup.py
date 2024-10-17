from setuptools import find_packages, setup

package_name = 'mobile_robot_controller'

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
    maintainer='tomasz_pietkun',
    maintainer_email='tomasz.pietkun.99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lqr_controller = mobile_robot_controller.lqr_controller:main',
            'mpc_controller = mobile_robot_controller.mpc_controller:main'
        ],
    },
)
