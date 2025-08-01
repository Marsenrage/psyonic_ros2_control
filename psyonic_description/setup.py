import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'psyonic_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf/*.xacro'))),  
        (os.path.join('share', package_name, 'urdf/meshes'), glob('urdf/meshes/*.STL')),  # Only if meshes exist
     ],
     
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hui',
    maintainer_email='hui@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
