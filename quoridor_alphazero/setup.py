from setuptools import find_packages, setup

package_name = 'quoridor_alphazero'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/play_vs_az.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bineth',
    maintainer_email='bineth.mandiv@gmail.com',
    description='AlphaZero-style (policy/value net + PUCT MCTS) Quoridor engine.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'ai_move_node = quoridor_alphazero.ai_move_node:main',
            'train = quoridor_alphazero.train:main',
        ],
    },
)
