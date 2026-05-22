from setuptools import find_packages, setup

package_name = 'upper_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doc049',
    maintainer_email='zhangyinrui00@gmail.com',
    description='zit6通信例程',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'config_setter = upper_examples.config_setter:main',
            'xbox_control = upper_examples.xbox_control:main',
            'image_viewer = upper_examples.image_viewer:main',
            'image_publisher = upper_examples.image_publisher:main',
            'heartbeat = upper_examples.heartbeat:main',
            'motion_control = upper_examples.motion_control:main',
            'gui = upper_examples.gui:main',
        ],
    },
)
