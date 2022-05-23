# Generate prefix from 4 levels up from env-hook directory (colcon ws install folder)
AMENT_CURRENT_PREFIX=$(ros2 pkg prefix leo_rover_description)

ament_prepend_unique_value GAZEBO_MODEL_PATH "$AMENT_CURRENT_PREFIX/share/leo_rover_description/models"
