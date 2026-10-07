from setuptools import find_packages, setup

package_name = 'mantis_commander_py'

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
    maintainer='vendicus',
    maintainer_email='vendicus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "teach_mode_node = mantis_commander_py.teach_mode:main",
            "detach_grip_service = mantis_commander_py.detach_grip:main"
        ],
    },
)
