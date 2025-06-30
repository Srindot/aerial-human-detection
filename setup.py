from setuptools import setup

package_name = 'yolo_realsense_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@email.com',
    description='YOLO human detection using RealSense camera frames',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_realsense_detector.yolo_detector:main'
        ],
    },
)
