from setuptools import setup

package_name = 'robocol_arm_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Camilo Gutierrez',
    maintainer_email='jc.gutierrezg@uniandes.edu.co',
    description='This package contains the ATTiny I2C control node for the Robocol Arm.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ATTinyMaster = robocol_arm_control.ATTinyMaster:main'
        ],
    },
)
