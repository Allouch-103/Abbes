from setuptools import find_packages, setup

package_name = 'humanoid_wbc'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yassine Allouch',
    maintainer_email='discoveryclubjunior@gmail.com',
    description='Whole-body control stack for Abbes (estimation, WBC QP, DCM/MPC).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_estimator = humanoid_wbc.estimation.estimator_node:main',
        ],
    },
)
