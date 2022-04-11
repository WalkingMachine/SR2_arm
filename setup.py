from setuptools import setup

package_name = 'sr2_arm'
robotiq_85 = 'sr2_arm/robotiq_85'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, robotiq_85],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walking',
    maintainer_email='wmaking@ens.etsmtl.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "start_server = sr2_arm.arm_service:main",
            "say = sr2_arm.arm_client:main",
        ],
    },
)
