#setup tools is python standard packaging library 
from setuptools import find_packages, setup
import os
#pattern matches
from glob import glob
package_name = 'drone_launch'

setup(
    name=package_name,
    version='0.0.0',
    #sca  source tree for directories with __init__/py and register them as installable python modules 
    packages=find_packages(exclude=['test']),
    data_files=[
        #register package with ament's package index so ros2 pkg list knows it's there
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        #copy package.xml to share/drone_launch
        ('share/' + package_name, ['package.xml']),
        #copy any file *launch.py to share/drone_launch, this is how ro2s launch finds launch file at runtime
        (os.path.join('share', package_name), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aidankwok',
    maintainer_email='aidankwok@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    #command line node executables
    entry_points={
        'console_scripts': [
        ],
    },
)
