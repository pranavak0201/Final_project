from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'final_project'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

    # Launch files
    data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')))

    # World files
    data_files.append((os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')))

    # Model files
    for root, dirs, files in os.walk('models'):
        model_files = [os.path.join(root, f) for f in files]
        if model_files:
            data_files.append((os.path.join('share', package_name, root), model_files))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='.', include=['final_project', 'final_project.*']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TurtleBot3 DRL project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train = final_project.train:main',
        ],
    },
)
