from glob import glob
from setuptools import find_packages, setup

package_name = 'trossen_arm_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trossen Robotics',
    maintainer_email='support@trossenrobotics.com',
    description='Examples for commanding the Trossen Arm Cartesian ros2_control controllers.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'cartesian_position_demo = trossen_arm_examples.cartesian_position_demo:main',
            'cartesian_button_press_demo = trossen_arm_examples.cartesian_button_press_demo:main',
        ],
    },
)
