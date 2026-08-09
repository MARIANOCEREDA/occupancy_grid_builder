from setuptools import find_packages, setup

package_name = 'segmentation_inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/yolo11n-seg.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='gmarianocereda@gmail.com',
    description='Segmentation inference package using Ultralytics YOLO models.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'segmentation_node = segmentation_inference.segmentation_node:main',
        ],
    },
)
