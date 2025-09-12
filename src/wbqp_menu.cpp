// wbqp_menu.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <iostream>
#include <string>

class WbqpMenuNode : public rclcpp::Node
{
public:
    WbqpMenuNode()
    : rclcpp::Node("wbqp_menu")
    {
        declare_parameter<std::string>("service_name", "/wbqp_controller/enable_qp");
        service_name_ = get_parameter("service_name").as_string();
        client_ = this->create_client<std_srvs::srv::SetBool>(service_name_);
    }

    void run() { runMenu(); } // public entry point

private:
    void printMenu()
    {
        std::cout << "\n=== WBQP MENU ===\n"
                  << "1) Enable QP\n"
                  << "0) Disable QP\n"
                  << "q) Quit\n\n";
    }

    void runMenu()
    {
        while (rclcpp::ok())
        {
            printMenu();
            std::cout << "Enter command: ";
            std::string cmd;
            if (!std::getline(std::cin, cmd)) { break; }

            if (cmd == "q" || cmd == "Q") { break; }
            if (cmd != "1" && cmd != "0")
            {
                std::cout << "Invalid choice.\n";
                continue;
            }

            bool enable = (cmd == "1");
            if (!client_->wait_for_service(std::chrono::seconds(2)))
            {
                std::cout << "Service [" << service_name_ << "] not available...\n";
                continue;
            }

            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = enable;

            auto future = client_->async_send_request(req);

            // SAFE now: object is fully managed by a shared_ptr
            if (rclcpp::spin_until_future_complete(this->shared_from_this(), future)
                != rclcpp::FutureReturnCode::SUCCESS)
            {
                std::cout << "Service call failed.\n";
                continue;
            }

            auto resp = future.get();
            std::cout << (resp->success ? "[OK] " : "[ERR] ") << resp->message << "\n";
        }
    }

private:
    std::string service_name_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WbqpMenuNode>();
    node->run();                   // <-- run after construction
    rclcpp::shutdown();
    return 0;
}
