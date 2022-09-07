#include <rclcpp/rclcpp.hpp> // for rclcpp
#include <image_transport/image_transport.hpp> // for image_transport
#include <sensor_msgs/image_encodings.hpp> // for sensor_msgs::image_encodings
#include <boost/asio.hpp> // for boost::asio, obviously

namespace ip = boost::asio::ip;

class UDPSend : public rclcpp::Node {
private:
    image_transport::Subscriber imageSubscriber;
    boost::asio::io_service io_service;
    ip::udp::socket socket;
    ip::udp::endpoint remote_endpoint;

public:
    UDPSend() : rclcpp::Node("UDPSend"), socket(io_service) {
        const std::string topic = this->declare_parameter("topic", "image");
        const std::string transport = this->declare_parameter("transport", "raw");
        RCLCPP_INFO(this->get_logger(),
            "Subscribing to %s using transport %s.", topic.c_str(), transport.c_str()
        );
        imageSubscriber = image_transport::create_subscription(this, topic,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {this->imageCb(std::move(msg));},
            transport
        );
        const std::string address = this->declare_parameter("address", "127.0.0.1");
        const int port = this->declare_parameter("port", 9000);
        RCLCPP_INFO(this->get_logger(),
            "Preparing to send to %s:%d.", address.c_str(), port
        );
        socket.open(ip::udp::v4());
        remote_endpoint = ip::udp::endpoint(ip::address::from_string(address), port);
        socket.connect(remote_endpoint);
    }

    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (msg->encoding != sensor_msgs::image_encodings::RGB8) {
            RCLCPP_ERROR(this->get_logger(),
                "Can only handle %s. This is %s.", sensor_msgs::image_encodings::RGB8, msg->encoding.c_str()
            );
            return;
        }
        try {
            std::string header("P6 " + std::to_string(msg->width) + " " + std::to_string(msg->height) + " 255 ");
            socket.send(boost::asio::buffer(header.c_str(), header.size()));
            const std::vector<unsigned char> & data = msg->data;
            const size_t mtu = 1500;
            for (size_t offset = 0; offset < data.size() ; offset += mtu) {
                const auto begin = data.data() + offset;
                const auto length = std::min(data.size() - offset, mtu);
                socket.send(boost::asio::buffer(begin, length));
            }
        } catch (boost::system::system_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPSend>());
    rclcpp::shutdown();
    return 0;
}
