from setuptools import setup

package_name = 'quadricslam_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lachness',
    maintainer_email='lnich13@live.com.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system = quadricslam_ros.quadricslam_ros:main',
            'dataset_publisher = quadricslam_ros.dataset_publisher:main',
            'webcam_publisher = quadricslam_ros.webcam_publisher:main',
        ],
    },
)
