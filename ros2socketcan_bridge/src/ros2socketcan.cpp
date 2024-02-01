#include "ros2socketcan_bridge/ros2socketcan.h"
#include <string>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

Ros2SocketCan::Ros2SocketCan()
    : Node("ros2socketcan_bridge"), stream(ios), signals(ios, SIGINT, SIGTERM)
{
    init();
}

void Ros2SocketCan::init()
{
    declare_parameter("can_socket", "can0");
    std::string canname = get_parameter("can_socket").as_string();

    topicname_receive_   << "CAN/" << canname.c_str() << "/" << "receive";
    topicname_transmit_  << "CAN/" << canname.c_str() << "/" << "transmit";

    rclcpp::executors::MultiThreadedExecutor exec;

    publisher_ 		= this->create_publisher<can_msgs::msg::Frame>(topicname_receive_.str(), 10);
    subscription_ 	= this->create_subscription<can_msgs::msg::Frame>(
                            topicname_transmit_.str(), 10,
                            std::bind(&Ros2SocketCan::canPublisher, this, _1));

    strcpy(ifr.ifr_name, canname.c_str());
    ioctl(natsock, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(natsock,(struct sockaddr *)&addr,sizeof(addr))<0)
    {
        perror("Error in socket bind");
    }

    stream.assign(natsock);

    RCLCPP_INFO(rclcpp::get_logger("socketcan_bridge"),
                    "ROS2 to CAN-Bus topic: %s", subscription_->get_topic_name());
    RCLCPP_INFO(rclcpp::get_logger("socketcan_bridge"),
                    "CAN-Bus to ROS2 topic: %s", publisher_->get_topic_name());

    stream.async_read_some(
        boost::asio::buffer(
            &rec_frame, sizeof(rec_frame)),
            std::bind(&Ros2SocketCan::canListener,
            this,std::ref(rec_frame),std::ref(stream)));
    signals.async_wait(std::bind(&Ros2SocketCan::stop, this));

    boost::system::error_code ec;
    std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
    std::thread bt(std::bind(run, &ios));
    bt.detach();
}

void Ros2SocketCan::stop()
{
    RCLCPP_INFO(rclcpp::get_logger("socketcan_bridge"),
         "End of Listener Thread. Please press strg+c again to stop the whole program.");
    ios.stop();
    signals.clear();
}

Ros2SocketCan::~Ros2SocketCan()
{
    RCLCPP_INFO(rclcpp::get_logger("socketcan_bridge"), "End of Publisher Thread. \n");
}

void Ros2SocketCan::canSend(const can_msgs::msg::Frame msg)
{
    struct can_frame frame1;

    frame1.can_id = msg.id;

    if (msg.is_extended)
    {
        frame1.can_id  = frame1.can_id + CAN_EFF_FLAG;
    }

    if (msg.is_error)
    {
        frame1.can_id  = frame1.can_id + CAN_ERR_FLAG;
    }

    if (msg.is_rtr)
    {
        frame1.can_id  = frame1.can_id + CAN_RTR_FLAG;
    }

    frame1.can_dlc = msg.dlc;

    for(int i=0;i<(int)frame1.can_dlc;i++)
    {
        frame1.data[i] = msg.data[i];
    }

    std::stringstream out;
    out << std::string("S | ") << std::to_string(frame1.can_id) << std::string("| ");
    for (int j = 0; j < (int)frame1.can_dlc; j++)
    {
        out << std::to_string(frame1.data[j]) << std::string(" ");
    }
    out << std::endl;
    RCLCPP_INFO(this->get_logger(), out.str().c_str());


    stream.async_write_some(
        boost::asio::buffer(&frame1, sizeof(frame1)),
        std::bind(&Ros2SocketCan::canSendConfirm, this));
}


void Ros2SocketCan::canPublisher(const can_msgs::msg::Frame::SharedPtr msg)
{

    can_msgs::msg::Frame msg1;
    msg1.id  = msg->id;
    msg1.dlc = msg->dlc;
    msg1.is_extended = msg->is_extended;
    msg1.is_rtr = msg->is_rtr;
    msg1.is_error = msg->is_error;
    msg1.data= msg->data;

    canSend(msg1);

}

void Ros2SocketCan::canSendConfirm(void)
{
    RCLCPP_DEBUG(rclcpp::get_logger("socketcan_bridge"), "Message sent");
}

void Ros2SocketCan::canListener(struct can_frame& rec_frame,
                                boost::asio::posix::basic_stream_descriptor<>& stream)
{

    can_msgs::msg::Frame frame;
    std::stringstream s;

    frame.id = rec_frame.can_id;
    frame.dlc = int(rec_frame.can_dlc);

    s << std::string("R | ") << std::to_string(rec_frame.can_id) << std::string(" | ");
    for (int i = 0; i < rec_frame.can_dlc; i++)
    {
        frame.data[i] = rec_frame.data[i];
        s << std::to_string(rec_frame.data[i]);
    }
    current_frame = frame;
    s << " | ";

    for (int j = 0; j < (int)rec_frame.can_dlc; j++)
    {
        s << std::to_string(rec_frame.data[j]) << " ";
    }
    s << std::endl;
    RCLCPP_INFO(get_logger(), s.str().c_str());
    publisher_->publish(frame);

    stream.async_read_some(
        boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
        std::bind(&Ros2SocketCan::canListener, this, std::ref(rec_frame), std::ref(stream)));

}

int main(int argc, char *argv[])
{
    std::cout << programdescr << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ros2SocketCan>();
    rclcpp::spin(node);
    // Free up any resources being used by the node
    rclcpp::shutdown();
    return 0;
}
