from setuptools import setup

package_name = 'rosquadricslam'

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
            'system = rosquadricslam.system:main',
            'dataset_publisher = rosquadricslam.dataset_publisher:main',
            'webcam_publisher = rosquadricslam.webcam_publisher:main',
        ],
    },
)
