from setuptools import setup

package_name = 'turtle_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtle_chase_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='menas-lap',
    maintainer_email='menas.icdl24@gmail.com',
    description='Turtle Chase Game',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'turtle_chase = turtle_pkg.turtle_chase:main'
        ],
    },
)
