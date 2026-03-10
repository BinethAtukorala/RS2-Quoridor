from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Bihan Sudusinghe',
    author_email='b.y.sudusinghe@gmail.com',
    description='Perception nodes for Quoridor robot',
    entry_points={
        'console_scripts': [
            'quoridor_perception = perception.quoridor_board_detection:main',
        ],
    },
)