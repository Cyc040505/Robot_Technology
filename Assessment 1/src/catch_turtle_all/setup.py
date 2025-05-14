from setuptools import setup

package_name = 'catch_turtle_all'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/catch_turtle.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ranqi',
    maintainer_email='3037857646@qq.com',
    description='Turtlesim catch all turtles project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_spawner = catch_turtle_all.turtle_spawner:main',
            'turtle_master = catch_turtle_all.turtle_master:main',
            'turtle_follower = catch_turtle_all.turtle_follower:main'  
        ],
    },
)
