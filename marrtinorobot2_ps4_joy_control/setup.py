from setuptools import setup

package_name = 'marrtinorobot2_ps4_joy_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'marrtinorobot2_ps4_joy_control.ps4_joy_control',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuo_nome',
    maintainer_email='tuo_email@example.com',
    description='Controllo robot con PS4 joystick',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'ps4_joy_control = marrtinorobot2_ps4_joy_control.ps4_joy_control:main'
        ],
    },
)
