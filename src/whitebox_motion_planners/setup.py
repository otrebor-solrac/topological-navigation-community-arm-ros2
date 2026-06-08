from setuptools import find_packages, setup

package_name = 'whitebox_motion_planners'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planning.launch.py']),
        ('share/' + package_name + '/config', ['config/planner_params.yaml', 'config/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roberto Carlos Vazquez Nava',
    maintainer_email='otrebor.solrac123@gmail.com',
    description='White-Box motion planning algorithms on topological manifolds (T^n)',
    license='CC BY-NC 4.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ROS 2 Node executables:
            # planner: launches whitebox_planner node (planning_node.py)
            'planner = whitebox_motion_planners.ros2.planning_node:main',
            # voxelizer: launches cspace_voxelizer node (cspace_publisher.py)
            'voxelizer = whitebox_motion_planners.ros2.cspace_publisher:main',
        ],
    },

)
