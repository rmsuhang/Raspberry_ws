from setuptools import find_packages, setup

package_name = 'ball_detection'

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
    maintainer='bubing3',
    maintainer_email='2606819673@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    #entry of the nodes
    entry_points={
        'console_scripts': [
            "ball_detection_node=ball_detection.ball_detection_node:main",
            "ball_detection_listener=ball_detection.ball_detection_listener:main"

        ],
    },

    
)
