// #include "rclcpp/rclcpp.hpp"
// //#include "std_msgs/msg/string.hpp"
// #include "interfaces_for_ik_service/srv/client_srv.hpp"
// #include "interfaces_for_ik_service/msg/client_msg.hpp"
// using std::placeholders::_1;

// class SubscriberNode : public rclcpp::Node {
// public:
//     SubscriberNode() : Node("subscriber_node") {
//         // Initialisierung des ROS 2 Service-Clients für Kinematik
//         kinematics_client_ = create_client<interfaces_for_ik_service::srv::ClientSrv>("kinematics");

//         // Warten auf den Service, um sicherzustellen, dass er verfügbar ist
//         while (!kinematics_client_->wait_for_service(std::chrono::seconds(1))) {
//             RCLCPP_INFO(get_logger(), "Kinematics service not available, waiting...");
//         }

//         // Erstellen des Subscribers, um Daten an den Service-Node zu senden
//         subscription_ = this -> create_subscription<interfaces_for_ik_service::msg::ClientMsg>(
//             "chatter",
//             10,
//             std::bind(&SubscriberNode::callback, this, std::placeholders::_1) // , std::placeholders::_1
//         );
//     }

// private:
//     void callback(const interfaces_for_ik_service::msg::ClientMsg::SharedPtr msg) { // std_msgs::msg::String::SharedPtr
       

//         /*  vermutlich nutzloser code
// ########################################
//         std::vector<std::string> pose_components;
//         std::istringstream iss(msg.x);
//         for (std::string component; std::getline(iss, component, ',');) {
//             pose_components.push_back(component);
//         } 

//          // Erstellen der Service-Anfrage mit den empfangenen Pose-Daten
//         auto request = std::make_shared<interfaces_for_ik_service::srv::ClientSrv>();
//         request->pose.x = std::stod(pose_components[0]);
//         request->pose.y = std::stod(pose_components[1]);
//         request->pose.z = std::stod(pose_components[2]);
// #########################################
//         vermutlich nutzloser code */

//         // Annahme: Die empfangene Daten enhalten Pose-Daten im Format float64 x,float64 y, float64 z
//         // todo: daten nicht in String umwandeln sondern direkt als soche belassen wie sie sind
//         // Erstellen der Service-Anfrage mit den empfangenen Pose-Daten

//   // Erstellen der Service-Anfrage mit den empfangenen Pose-Daten
// auto request = std::make_shared<interfaces_for_ik_service::srv::ClientSrv::Request>();
// request->x = msg->x;
// request->y = msg->y;
// request->z = msg->z;


// // Aufrufen des Kinematik-Service mit den Pose-Daten
//     auto result_future = kinematics_client_->async_send_request(request);

//     // Warten auf die Antwort des Services
//     auto result = result_future.get();
//     if (result) {
//         RCLCPP_INFO(get_logger(), "Kinematics service response received");
//         // Verarbeiten Sie die Antwort des Dienstes hier entsprechend
//     } else {
//         RCLCPP_ERROR(get_logger(), "Failed to call Kinematics service");
//     }



//     }

//     rclcpp::Client<interfaces_for_ik_service::srv::ClientSrv>::SharedPtr kinematics_client_;
//     rclcpp::Subscription<interfaces_for_ik_service::msg::ClientMsg>::SharedPtr subscription_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);

//     auto subscriber_node = std::make_shared<SubscriberNode>();

//     rclcpp::spin(subscriber_node);

//     rclcpp::shutdown();

//     return 0;
// }
