from setuptools import find_packages, setup

package_name = 'dobot_controller'

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
    maintainer='timevisao',
    maintainer_email='timevisao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_dobot_action = dobot_controller.move_dobot_action:main',
            'Control_teste = dobot_controller.Control_teste:main',
        ],
    },
)
