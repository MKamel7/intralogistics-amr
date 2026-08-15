from setuptools import find_packages, setup

package_name = 'amr_vda5050'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Kamel',
    maintainer_email='mkamel860@gmail.com',
    description='VDA 5050 vehicle interface over MQTT.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vda5050_bridge = amr_vda5050.vda5050_bridge:main',
        ],
    },
)
