from setuptools import find_packages, setup

package_name = 'data_collect'

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
    maintainer='ee512',
    maintainer_email='oyinggaio@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_collect_node = data_collect.camera_collect_node:main',
            'launch_info_collect_node = data_collect.launch_info_collect_node:main'
        ],
    },
)
