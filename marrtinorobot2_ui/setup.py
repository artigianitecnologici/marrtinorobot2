from setuptools import find_packages, setup

package_name = 'marrtinorobot2_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (f'share/{package_name}/launch', ['launch/ui_node.launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ferrarini fabio',
    maintainer_email='ferrarini09@gmail.com',
    description='module tts',
    license='',
    entry_points={
        'console_scripts': [
            'play_face = marrtinorobot2_ui.play_face:main',
            'play_face_cars =  marrtinorobot2_ui.play_face_cars:main',
            'ui_node = marrtinorobot2_ui.ui_node:main',
        ],
    },
)


