from setuptools import find_packages, setup
from glob import glob

package_name = 'burgerbot_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/maps',   glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aimen',
    maintainer_email='aimene.ch.derradji@gmail.com',
    description='Burgerbot Nav2 navigation',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
