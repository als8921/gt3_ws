from setuptools import setup
import os
from glob import glob

package_name = 'mobile_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lmc',
    maintainer_email='als8921@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "control = mobile_control.MobileController:main",
            "calculate_angle = mobile_control.CreateCommand:main",
            "scan = mobile_control.PCDFileHandler:main",
            "battery = mobile_control.BatteryCheck:main"
        ],
    },
)
