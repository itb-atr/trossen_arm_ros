# Cartesian wrench command for trossen_arm_controllers/CartesianExternalEffortController.
# The wrench is interpreted by the hardware interface according to interpolation_space.

string INTERPOLATION_SPACE_JOINT=joint
string INTERPOLATION_SPACE_CARTESIAN=cartesian

std_msgs/Header header
geometry_msgs/Wrench wrench
float64 goal_time
string interpolation_space
