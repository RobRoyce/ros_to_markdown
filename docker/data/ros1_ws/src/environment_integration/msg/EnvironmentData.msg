# This message is used to send environment data to the robot
float64[] map_data # Environmental map data
string frame_id# Frame ID of the map data

# If the map data is not available, the robot should use the frame_id to determine the position of the robot.
# The frame_id is a string that uniquely identifies the frame of reference for the map data.
# The frame_id is typically a string that represents the name of the frame, such as "map" or "odom".
