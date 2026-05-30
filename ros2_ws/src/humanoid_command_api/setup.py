from setuptools import find_packages, setup
from glob import glob

package_name = 'humanoid_command_api'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='allouch-103',
    maintainer_email='yassine.allouch@insat.ucar.tn',
    description='Humanoid command API: actions and services for the LLM layer.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_motions   = humanoid_command_api.test_motions:main',
            'command_server = humanoid_command_api.command_server_node:main',
            'llm_dispatcher = humanoid_command_api.llm_dispatcher_node:main',
            'web_ui         = humanoid_command_api.web_ui_node:main',
        ],
    },
)