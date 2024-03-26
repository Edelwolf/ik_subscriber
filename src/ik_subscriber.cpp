// colcon build --packages-select ik_subscriber
// source install/setup.bash
// ros2 run ik_subscriber ik_subscriber

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "ik_interfaces/srv/calc_ik.hpp"
#include <fstream>
#include <sstream>
#include <string>

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("subscriber_node") {
        kinematics_client_ = create_client<ik_interfaces::srv::CalcIK>("kinematics");
        while (!kinematics_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for the kinematics service to be available...");
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SubscriberNode::check_file_and_request_ik, this));
    }

private:
    void check_file_and_request_ik() {
        std::string new_content = read_pose_file("/home/flehm/ros2_ws/src/ik_subscriber/src/position.txt"); // Pfad zur Pose-Datei anpassen
        if (new_content != current_file_content_) {
            current_file_content_ = new_content;
            auto pose = parse_pose_from_content(new_content);
            send_ik_request(pose);
        }
    }

    void send_ik_request(const geometry_msgs::msg::Pose& pose) {
        auto request = std::make_shared<ik_interfaces::srv::CalcIK::Request>();
        request->pose = pose;

        kinematics_client_->async_send_request(request,
            [this](rclcpp::Client<ik_interfaces::srv::CalcIK>::SharedFuture future) {
                auto response = future.get();
                if (response && !response->ik_joint_states.empty()) {
                    RCLCPP_INFO(this->get_logger(), "IK calculation successful, printing solutions:");
                    for (const auto& joint_state : response->ik_joint_states) {
                        if (joint_state.is_valid) {
                            RCLCPP_INFO(this->get_logger(), "Valid solution found:");
                            for (size_t i = 0; i < joint_state.name.size(); ++i) {
                                RCLCPP_INFO(this->get_logger(), "  %s: %f", joint_state.name[i].c_str(), joint_state.state[i]);
                            }
                        }
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to receive response from service or no solutions found.");
                }
            });
    }

    std::string read_pose_file(const std::string& file_path) {
        std::ifstream file(file_path);
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    geometry_msgs::msg::Pose parse_pose_from_content(const std::string& content) {
        geometry_msgs::msg::Pose pose;
        std::istringstream stream(content);
        std::string line;
        while (std::getline(stream, line)) {
            std::istringstream line_stream(line);
            std::string key;
            double value;
            if (getline(line_stream, key, ':') && (line_stream >> value)) {
                if (key == "position_x") pose.position.x = value;
                else if (key == "position_y") pose.position.y = value;
                else if (key == "position_z") pose.position.z = value;
                else if (key == "orientation_x") pose.orientation.x = value;
                else if (key == "orientation_y") pose.orientation.y = value;
                else if (key == "orientation_z") pose.orientation.z = value;
                else if (key == "orientation_w") pose.orientation.w = value;
            }
        }
        return pose;
    }

    rclcpp::Client<ik_interfaces::srv::CalcIK>::SharedPtr kinematics_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string current_file_content_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
