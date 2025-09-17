from setuptools import find_packages, setup

package_name = 'weather_pkg'

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
    maintainer='menas-lap',
    maintainer_email='menas.icdl24@gmail.com',
    description='Weather Monitoring System',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'temprature_node = weather_pkg.temprature_node:main',
          'humidity_node = weather_pkg.humidity_node:main',
          'pressure_node = weather_pkg.pressure_node:main',
          'monitor_node = weather_pkg.monitor_node:main',
        ],
    },
)
