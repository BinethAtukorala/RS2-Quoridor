from setuptools import find_packages, setup

package_name = 'quoridor_game'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bineth',
    maintainer_email='bineth@todo.todo',
    description='Quoridor game control subsystem for UR3e robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_manager = quoridor_game.state_manager:main',
            'move_decision = quoridor_game.move_decision:main',
            'user_interface = quoridor_game.user_interface:main',
            'web_interface = quoridor_game.web_interface:main',
        ],
    },
)
