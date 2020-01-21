from setuptools import setup
import glob

package_name = 'rmf_core_tools'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.xml')),
        ('share/' + package_name + '/maps', glob.glob('maps/*')),
        ('share/' + package_name + '/config', glob.glob('config/*')),
        ('share/' + package_name + '/robots', glob.glob('robots/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bhan',
    maintainer_email='cnboonhan94@gmail.com',
    description='Tools for working with multi-robot systems of rmf_core',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shell_robot = rmf_core_tools.shell_robot:main'
        ],
    },
)
