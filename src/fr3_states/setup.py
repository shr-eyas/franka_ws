from setuptools import find_packages, setup

package_name = 'fr3_states'

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
    maintainer='sophia',
    maintainer_email='shreyas.kumar@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jacobian_publisher = fr3_states.jacobian_publisher:main',
            'pose_publisher = fr3_states.pose_publisher:main',
        ],
    },
)
