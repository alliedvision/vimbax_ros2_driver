#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>


int main(int argc, char *argv[]) 
{
    auto const args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    auto const stream_count = std::stoi(args[1]);

    auto node = rclcpp::Node::make_shared("_asynchronous_grab_cpp");

    int64_t last_frame_id = -1;
    int64_t frames_missing = 0;
    int64_t frame_count = 0;


    auto subscription = image_transport::create_subscription(node.get(), "/vimbax_camera_test/image_raw",  
        [&](sensor_msgs::msg::Image::ConstSharedPtr imgmsg) {
            auto const frame_id = std::stol(imgmsg->header.frame_id);
            auto const missing = frame_id - last_frame_id - 1;

            if (missing > 0) {
                frames_missing += missing;
                std::cout << "Detected " << missing << " missing frames!" << std::endl;
            }


            std::cout << "Got frame " << frame_id << std::endl;

            last_frame_id = frame_id;
            frame_count++;
            if (frame_count >= stream_count)  {
                rclcpp::shutdown();     
            }
        }, "raw");

    rclcpp::spin(node);

    std::cout << "Missed " << frames_missing << "/" << frame_count << " frames" << std::endl;
}