ROS 2.0 to CAN Bridge  {#ros2candoc}
---------------------

# Overview
For easy access to the CAN bus a bridge between ROS 2.0 topics to the CAN bus was implemented. The CAN Bus is included to the Linux operating system as network socket with the socket CAN driver. Therefore the CAN bus is included to the software similar to an Ethernet socket using the “asio” library. The node functions as a bidirectional bridge and listens to the CAN bus and publishes the received messages to a ROS 2.0 topic called:

- "CAN/" + CAN socket name +  "/receive"

Equally the messages on a ROS 2.0 topic called:

- "CAN/" + CAN socket name +  "/transmit"

are monitored and forwarded to the CAN bus. Examples for teh topic names are:

- "CAN/can0/receive"
- "CAN/can0/transmit"

The topic names are structured in 2 field names and the transmit and receive topic. The first field name “CAN” identifies the topic within ROS2.0 as a CAN Topic. The ‘CAN_bus_name’  identifies the CAN bus within an building block, because multiple CAN buses can be connected. A ROS 2.0 to CAN Bridge node is always coupled to one BUS bus. The name of the CAN Bus can be adjusted with a command line argument, e.g. “ros2can_bridge can1”.

# Build
Use "ament build --only-packages ros2socketcan_bridge" to build the package.

# Usage
The ros2can bridge is launched using the command: "ros2can_bridge". It will automatically start the CAN Bridge with the CAN socket "can0". To change the socket the CAN bridge add a command line argument, e.g. "ros2can_bridge can1".

# ROS2 Message Type
The message definitions are done in the can_msgs package. It provides the "frame" message type for topics. 


## Frame:
```
std_msgs/Header header
uint32 id
uint8 dlc
uint8[8] data
uint8 err
uint8 rtr
uint8 eff
```
