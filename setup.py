from setuptools import setup
import os
from glob import glob

package_name = 'final_project'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Include launch files
    data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')))
    
    # Include world files
    data_files.append((os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')))
    
    # Include model files recursively
    model_dirs = []
    for root, dirs, files in os.walk('models'):
        # Get all files in the current directory
        model_files = [os.path.join(root, f) for f in files]
        if model_files:
            # Add the directory and its files
            data_files.append((os.path.join('share', package_name, root), model_files))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TurtleBot3 dynamic Gazebo world project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)