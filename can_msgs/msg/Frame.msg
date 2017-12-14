std_msgs/Header header
uint32 id
uint8 dlc
uint8[8] data
uint8 err
uint8 rtr
uint8 eff

## @file Frame.msg
# @ingroup ros2can_bridge
# @author Philipp Wuestenberg
# @brief message type to publish can messages to ROS topic
# @param id CAN message id (11/29 bit)
# @param dlc CAN data size in bytes
# @param data CAN data
# @param err error flag bit (0 = data frame, 1 = error message)
# @param rtr remote transmission request flag (1 = rtr frame)
# @param eff frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
