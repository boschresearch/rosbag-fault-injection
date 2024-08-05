import os
from glob import glob
from setuptools import setup

package_name = 'num_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='MinHee.Jo@de.bosch.com',
    description='Number Generator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generator = num_generator.generator:main',
            'multiplier = num_generator.multiplier:main',
            'feedback = num_generator.feedback:main',
            'initialiser = num_generator.initialiser:main',
        ],
    },
)
