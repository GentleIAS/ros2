from setuptools import find_packages, setup

package_name = 'python_package'

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
    maintainer='sun',
    maintainer_email='gentleias78698@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_python_node = python_package.python_node:main',
            #"ros2_python_node"    创建后的可执行文件名字
            #"python_package.python_node:main"    包名.文件名:函数名
        ],
    },
)
