from setuptools import find_packages, setup

package_name = 'system_manager'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mode_manager = system_manager.mode_manager:main',
            # ★ 이 줄을 추가합니다. (실행파일명 = 패키지명.파일명:함수명)
            'mode_manager_backup2 = system_manager.mode_manager_backup2:main',
        ],
    },
)
