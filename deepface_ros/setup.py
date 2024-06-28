from setuptools import find_packages, setup

package_name = 'deepface_ros'

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
    maintainer='Samuele Sandrini',
    maintainer_email='samuele.sandrini@polito.it',
    description='DeepFace Wrapper for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepface_node = deepface_ros.deepface_node:main',
        ],
    },
)
