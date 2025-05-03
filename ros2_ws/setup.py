from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rnr_map'

# Iterate through all the files and subdirectories
# to build the data files array
def generate_data_files(share_path, dir):
    data_files = []
    
    for path, _, files in os.walk(dir):
        list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (f'share/{package_name}/launch', glob(f'launch/*launch.[pxy][yma]*')),
        (f'share/{package_name}/rviz', glob(f'rviz/*')),
        # (f'share/{package_name}/models', glob(f'models/*')),
        # (f'share/{package_name}/world', [_ for _ in glob(f'world/**/*', recursive=True) if os.path.isfile(_)] ),
        # [(os.path.join('share', package_name, os.path.split(path)[0]), [path]) for path in glob('world/**', recursive=True)]
    ] + generate_data_files(f'share/{package_name}/', 'models')
    + generate_data_files(f'share/{package_name}/', 'world'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bachnguyen',
    maintainer_email='ngb1998@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "extract_data = rnr_map.extract_data:main" 
        ],
    },
)
