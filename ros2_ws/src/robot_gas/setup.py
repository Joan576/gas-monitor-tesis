from setuptools import setup

package_name = 'robot_gas'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_gas.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot_gas.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebas',
    maintainer_email='sebas@example.com',
    description='Gas detection node and publisher',
    license='MIT',
    entry_points={
	    'console_scripts': [
            'gas_publisher = robot_gas.gas_publisher:main',
            'gas_to_influx = robot_gas.gas_to_influx:main',
            'gas_xy_to_influx = robot_gas.gas_xy_to_influx:main',
   	 ],
    },

)
