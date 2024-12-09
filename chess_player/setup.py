from setuptools import setup

package_name = 'chess_player'
setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Ferguson',
    maintainer_email='mike@vanadiumlabs.com',
    description='Executive for AAAI Chess 2011',
    license='GPL',
)
