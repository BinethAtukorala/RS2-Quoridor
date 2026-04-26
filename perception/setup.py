from setuptools import setup, find_packages

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),   # finds the inner 'perception' folder
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyrealsense2', 'numpy', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    author='Bihan Sudusinghe',
    author_email='b.y.sudusinghe@gmail.com',
    description='Quoridor perception package',
    entry_points={
        'console_scripts': [
            'grid_detector_node = perception.grid_detector_node:main',
            'circle_detector_node = perception.circle_detector_node:main',
            'coordinate_node = perception.coordinate_node:main',
            'camera_node = perception.camera_node:main',
            'ar_node = perception.ar_node:main',
            'perception_node = perception.perception_node:main',
        ],
    },
)