from setuptools import find_packages, setup

package_name = 'my_4dof_arm'

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
    maintainer='karthik',
    maintainer_email='karthikmanoranjan11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'send_joint_angles = my_4dof_arm.send_joint_angles:main'
        ],
    },
)