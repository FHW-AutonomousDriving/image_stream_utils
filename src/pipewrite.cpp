#include <rclcpp/rclcpp.hpp> // for rclcpp
#include <image_transport/image_transport.hpp> // for image_transport
#include <sensor_msgs/image_encodings.hpp> // for sensor_msgs::image_encodings

class Pipewrite : public rclcpp::Node {
private:
    image_transport::Subscriber imageSubscriber;

public:
    Pipewrite() : rclcpp::Node("Pipewrite") {
        const std::string topic = this->declare_parameter("topic", "image");
        const std::string transport = this->declare_parameter("transport", "raw");
        RCLCPP_INFO(this->get_logger(),
            "Subscribing to %s using transport %s.", topic.c_str(), transport.c_str()
        );
        imageSubscriber = image_transport::create_subscription(this, topic,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {this->imageCb(std::move(msg));},
            transport
        );
    }

    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (msg->encoding != sensor_msgs::image_encodings::RGB8) {
            RCLCPP_ERROR(this->get_logger(),
                "Can only handle %s. This is %s.", sensor_msgs::image_encodings::RGB8, msg->encoding.c_str()
            );
            return;
        }
        // hooray for poor man's ppm output and ffmpeg's ppm_pipe
        std::cout << "P6 " << msg->width << " " << msg->height << " 255 ";
        std::cout.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pipewrite>());
    rclcpp::shutdown();
    return 0;
}
