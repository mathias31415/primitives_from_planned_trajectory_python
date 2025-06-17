from setuptools import find_packages, setup

package_name = 'primitives_from_planned_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['primitives_from_planned_trajectory', 'primitives_from_planned_trajectory.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mathias Fuhrer',
    maintainer_email='mathias.fuhrer@b-robotized.com',
    description='Package to approximate a planned trajectory using motion primitives such as PTP, LIN, and CIRC',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'process_trajectory_to_primitives = primitives_from_planned_trajectory.process_trajectory_to_primitives:main',
            'plot_saved_trajectory = primitives_from_planned_trajectory.plot_saved_trajectory:main',
        ],
    },
)
