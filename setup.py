from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bug2_nav'

# Initialize the data_files list with common entries
data_files_list = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y|m]'))),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
]

# --- Add this section to recursively install your models ---
# Define the source directory for models relative to setup.py
model_source_dir = 'models'

# Define the base installation directory for models
# This will be 'share/bug2_nav/models' in the install space
model_install_base = os.path.join('share', package_name, model_source_dir)

# Check if the source models directory exists to avoid errors if it's empty or missing
if os.path.exists(model_source_dir):
    # os.walk traverses the directory tree, yielding (dirpath, dirnames, filenames)
    for root, dirs, files in os.walk(model_source_dir):
        # Calculate the relative path from the model_source_dir to the current 'root'
        # Example: if root is 'models/building_1', relative_path_in_models becomes 'building_1'
        #          if root is 'models', relative_path_in_models becomes '.'
        relative_path_in_models = os.path.relpath(root, model_source_dir)
        
        # Construct the full destination directory path in the install space for the current files
        # Example: 'share/bug2_nav/models' or 'share/bug2_nav/models/building_1'
        destination_dir = os.path.join(model_install_base, relative_path_in_models)
        
        # Add each file found in the current directory to the data_files_list
        for file in files:
            source_file_path = os.path.join(root, file) # e.g., 'models/building_1/model.sdf'
            data_files_list.append((destination_dir, [source_file_path]))
# -----------------------------------------------------------

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files_list, # Use the prepared list that now includes models
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phumint',
    maintainer_email='phumint@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_scan = bug2_nav.laser_scan:main',
            'right_wall_following = bug2_nav.right_wall_following:main',
            'bug0_building_0 = bug2_nav.bug0_building_0:main',
            'bug0_building_2 = bug2_nav.bug0_building_2:main',
            'attract = bug2_nav.attract:main',
            'bug1_building_0 = bug2_nav.bug1_building_0:main',
        ],
    },
)