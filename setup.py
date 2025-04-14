from setuptools import setup

package_name = 'i2c_temperature'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Erin Linebarger',
    maintainer_email='erin@robotics88.com',
    description='ROS 2 node that reads temperature from a SEN0546 sensor over I2C and publishes it.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_node = i2c_temperature.temperature_node:main'
        ],
    },
)
