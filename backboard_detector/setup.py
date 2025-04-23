from setuptools import find_packages, setup

package_name = 'backboard_detector'

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
    maintainer='kaoru',
    maintainer_email='kaoru.yoshida@keio.jp',
    description='pointcloudから平面のローカル座標を返すノード',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'backboard_detector = backboard_detector.backboard_detector:main'
        ],
    },
)
