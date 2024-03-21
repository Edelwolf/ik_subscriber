#include "rclcpp/rclcpp.hpp"
// #include "interfaces_for_ik_service/srv/client_srv.hpp"
// #include "interfaces_for_ik_service/msg/client_msg.hpp"
#include <geometry_msgs/msg/pose.hpp>

// #include <ik_solver/interfaces/msg/.h>
#include "ik_interfaces/srv/calc_ik.hpp"


using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("subscriber_node") {
        // Initialisierung des ROS 2 Service-Clients für Kinematik
        kinematics_client_ = create_client<ik_interfaces::srv::CalcIK>("kinematics");

        // Warten auf den Service, um sicherzustellen, dass er verfügbar ist
        while (!kinematics_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Kinematics service not available, waiting...");
        }

        // Erstellen des Subscribers, um Nachrichten vom Typ 
        // interfaces_for_ik_service::msg::ClientMsg auf 
        // dem Thema "chatter" zu abonnieren
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "chatter",
            10,
            std::bind(&SubscriberNode::callback, this, std::placeholders::_1)
        );

        // TEST

                // Timer überprüft alle 1 Sekunde die Datei 
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FileWatcherNode::check_file_and_request_ik, this));

        // Aktuellen Dateiinhalt initial leer setzen
        current_file_content_ = "";

        // TEST 
    }

private:
void callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
// kleine änderung 
        // TEST
   std::string new_content = read_pose_file("path/to/your/pose.txt");
        if (new_content != current_file_content_) {
            // Datei hat sich geändert, aktualisiere den aktuellen Inhalt und sende Anfrage
            current_file_content_ = new_content;
            send_ik_request(parse_pose_from_content(new_content));
        }
    }

    std::string read_pose_file(const std::string& file_path) {
        std::ifstream file(file_path);
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }


        // TEST 

    // Pose-Nachricht zuweisen
    auto request = std::make_shared<ik_interfaces::srv::CalcIK::Request>();
    request->pose = *msg; 

    // Asynchroner Aufruf des Service
    auto result_future = kinematics_client_->async_send_request(request,
        [this](rclcpp::Client<ik_interfaces::srv::CalcIK>::SharedFuture future) {
            auto response = future.get();
            // Antwort der berechneten Gelenkzustände ausgeben
            if(response) {
                RCLCPP_INFO(this->get_logger(), "Service call successful.");
                // Iteration über die ik_joint_states und Ausgabe
                for (const auto& joint_state : response->ik_joint_states) {
                    RCLCPP_INFO(this->get_logger(), "Joint State:");
                    // Ausgabe der Gelenknamen und -zustände
                    for (size_t i = 0; i < joint_state.name.size(); ++i) {
                        RCLCPP_INFO(this->get_logger(), "  %s: %f", joint_state.name[i].c_str(), joint_state.state[i]);
                    }
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive response from service.");
            }
        });
}


    rclcpp::Client<ik_interfaces::srv::CalcIK>::SharedPtr kinematics_client_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto subscriber_node = std::make_shared<SubscriberNode>();

    rclcpp::spin(subscriber_node);

    rclcpp::shutdown();

    return 0;
}



