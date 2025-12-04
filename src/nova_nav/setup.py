from setuptools import setup
from glob import glob

package_name = 'nova_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maulana Muammar',
    maintainer_email='maulana.muammar@binus.ac.id',
    description='Nova agv URDF + launch',
    license='MIT',
    entry_points={
        'console_scripts': [
            'agv_bridge_node = nova_nav.agv_bridge_node:main',
        ],
    },
)
