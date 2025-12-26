from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_client_cluster2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # ÀÚµ¿ ÆÐÅ°Áö Å½»ö
    data_files=[
        # ROS2 ÆÐÅ°Áö ÀÎµ¦½º µî·Ï
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml º¹»ç
        ('share/' + package_name, ['package.xml']),

        # model Æú´õ ÀüÃ¼ º¹»ç
        (
            os.path.join('share', package_name, 'model'),
            glob('camera_client_cluster2/model/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='Camera Client Cluster 2 - clean separated package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # A-Pi ³ëµå
            'camera_client_node2 = camera_client_cluster2.camera_client_node2:main',

            # A-pi backup
            'camera_client_node_backup = camera_client_cluster2.camera_client_node_backup:main',
            
            # A-pi color_detect experiment
            'camera_client_node_color = camera_client_cluster2.camera_client_node_color:main',
            # A-pi wook experiment
            'camera_client_node3 = camera_client_cluster2.camera_client_node3:main',

        ],
    },
)

