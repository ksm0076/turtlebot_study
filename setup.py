from setuptools import setup

package_name = 'my_pkg'

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
    maintainer='ksm',
    maintainer_email='ksm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_pkg.my_node:main',
            'nav_robot = my_pkg.nav_robot:main',
            'person_segmentation = my_pkg.person_segmentation:main',
            'nav_person = my_pkg.nav_person:main',
            'temp = my_pkg.temp:main',
            'quart_test = my_pkg.quart_test:main',
            'measure_distance = my_pkg.measure_distance:main',
            'measure_command = my_pkg.measure_command:main',
            'distance_gui = my_pkg.distance_gui:main',
        ],
    },
)
