from setuptools import find_packages, setup

package_name = 'quoridor_ai_move'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ai_move.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bineth',
    maintainer_email='bineth.mandiv@gmail.com',
    description='CNN + Deep Q-Learning move engine for Quoridor (drop-in replacement for quoridor_move_decision).',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'ai_move_node = quoridor_ai_move.ai_move_node:main',
            'train = quoridor_ai_move.train:main',
            'train_vs_model = quoridor_ai_move.train_vs_model:main',
        ],
    },
)
