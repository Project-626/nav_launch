from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'nav_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('configs/*')),
<<<<<<< HEAD
        (os.path.join('share', package_name), glob('meshes/*')),
        (os.path.join('share', package_name), glob('urdf/*'))
=======
        (os.path.join('share', package_name), glob('urdf/*')),
>>>>>>> 103913f1d56b169cbc99e5dfb8fdceeb03a0a9b9
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='max.schik@googlemail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
