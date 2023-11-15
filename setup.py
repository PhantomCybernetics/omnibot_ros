from setuptools import find_packages, setup

package_name = 'omnibot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mirek Burkon',
    maintainer_email='mirek@phntm.io',
    description='Omnibot Firmware',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'firmware = omnibot.firmware:main'
        ],
    },
)
