cmake_minimum_required(VERSION 3.8)
project(bug2_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

# Install directories containing resources (launch, worlds)
# These seem to be working fine for individual files
install(
  DIRECTORY
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# Explicitly install the CONTENTS of the models directory.
# The trailing slash on 'models/' is crucial here.
install(
  DIRECTORY models/ # <-- ADD A TRAILING SLASH HERE
  DESTINATION share/${PROJECT_NAME}/models # <-- Ensure this is the target models directory
)

# Call the ament_package() macro to configure the package for ROS 2
ament_package()