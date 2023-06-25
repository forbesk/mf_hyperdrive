from setuptools import setup

package_name = 'mf_hyperdrive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'mf_hyperdrive.ms5837'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PCA9685-driver', 'Jetson.GPIO', 'mypy', 'smbus2', 'pyserial', 'crc'],
    zip_safe=True,
    maintainer='Kevin Forbes',
    maintainer_email='forbesk9@gmail.com',
    description='Millennial Falcon Hyperdrive package. Combines various functions related to Hyperdrive PCB, including IMU (MPU6050), PWM generation (PCA9685), and depth sensing.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hyperdrive = mf_hyperdrive.hyperdrive:main',
        ],
    },
)
