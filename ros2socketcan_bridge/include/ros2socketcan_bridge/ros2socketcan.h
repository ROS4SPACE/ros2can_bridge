// Licensed to the Apache Software Foundation (ASF) under one
// or more contributor license agreements.  See the NOTICE file
// distributed with this work for additional information
// regarding copyright ownership.  The ASF licenses this file
// to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing,
// software distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.

/** @file Ros2SocketCan.h
 *
 *  @ingroup ROS2CAN_bridge
 *  @author Philipp Wuestenberg
 *  @brief  bidirectional ROS2 to CAN interface with topics and service
 */

#ifndef __ros2_socketcan_H__
#define __ros2_socketcan_H__

#include <linux/can/raw.h>

#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"


const std::string version = "1.00 from: " + std::string(__DATE__) + " " + std::string(__TIME__);
const std::string programdescr = "ROS 2 to CAN-Bus Bridge\nVersion: " + version;

/**
 * @brief The ros2socketcan bridge connects a canbus with the ROS2 topic system.
 * @details A nodes is provided, which provides the bridge from a ROS topic to the CAN bus and
 *          from the CAN bus to a ROS topic. The node functions as a bidirectional bridge and
 *          provides service to publish a message and receive the answer with the fitting
 *          message id.
 *
 */
class Ros2SocketCan : public rclcpp::Node
{
    public:
        /**
         * @brief constructor for Ros2SocketCan class
         * @details Within the constructor the topic and service naming is done.
         */
        Ros2SocketCan();

        /**
         * @brief Within the init() fucntin the ROS and CAN setup is done.
         * @details Within the Init() function the ROS2 publisher, subscriber and
         *          the service server is initialized. In addition the socketcan interface is
         *          configured and assigned to the socket. The Init function is necessary as
         *          the topics need a fully constructed node class to be added to.
         */
        void init();
        /**
         * @brief destructor
         */
        ~Ros2SocketCan();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;

        can_msgs::msg::Frame current_frame;

        /**
         * @brief The CanSendConfirm function is needed by the .async_write_some function and is
         *        called as confirmation for a successfull send process.
         */
        void canSendConfirm();

        /**
         * @brief The CanPublisher is listening to a ROS2 Topic and calls the CanSend Method.
         */
        void canPublisher(const can_msgs::msg::Frame::SharedPtr msg);

        /**
         * @brief The CanSend method sends a ROS message to the CAN bus.
         * @details The CanSend function is Called by the CanPublisher and ther ros2can_srv.
         *          It converts the ROS message to a can_frame and adds the CAN Flags
         *          to the message ID.
         */
        void canSend(const can_msgs::msg::Frame msg);

        /**
         * @brief The CanListener listens to the CAN Bus and publishes the message to a ROS2 Topic.
         * @details The CanListener function is Called by the .async_read_some when a Message is
         *          received on the Can Socket. It converts the message to a ROS Message and
         *          publishes it to a ROS2 Topic. Afterwards .async_read_some must be called again
         *          to wait for further CAN Messages.
         */
        void canListener(struct can_frame& rec_frame,
                         boost::asio::posix::basic_stream_descriptor<>& stream);

        /**
         * @brief The Stop method is needed as the interuped handler must be configered
         *        to the asio libary.
         */
        void stop();

        boost::asio::io_service ios;
        boost::asio::posix::basic_stream_descriptor<> stream;
        boost::asio::signal_set signals;

        struct sockaddr_can addr;
        struct can_frame frame;
        struct can_frame rec_frame;
        struct ifreq ifr;

        int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        std::stringstream topicname_receive_;
        std::stringstream topicname_transmit_;
};
#endif
