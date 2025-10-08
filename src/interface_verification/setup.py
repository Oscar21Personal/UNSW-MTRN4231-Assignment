from setuptools import find_packages, setup

package_name = 'interface_verification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitchell',
    maintainer_email='mitch.torok@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inventory_verification = interface_verification.inventory_verification:main',
            'brain_verification = interface_verification.brain_verification:main',
            'perception_verification = interface_verification.perception_verification:main',
        ],
    },
)
