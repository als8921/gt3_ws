from setuptools import setup
import os
from glob import glob

package_name = 'mobile_control'
submodules = 'mobile_control/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
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
            "twist_transform = mobile_control.CtrlCmdtoTwist:main",
            "control = mobile_control.positionCalculate:main",
            "command = mobile_control.commandPublisher:main",
        ],
    },
)
