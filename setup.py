from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'biped_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Include our package.xml file
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files (using * ensures .launch.py and others are caught)
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        
        # Include all URDF and Xacro files (using * catches meshes/stls too if present)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # Include all Config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manoja',
    maintainer_email='manoja@todo.todo',
    description='Biped robot description and walking controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'walking_controller = biped_description.scripts.walk:main',
        ],
    },
)
