#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <chrono>
#include <mutex>
#include <optional>

using namespace std::chrono_literals;

class WbqpControllerNode : public rclcpp::Node, public std::enable_shared_from_this<WbqpControllerNode>
{
public:
    WbqpControllerNode()
    : rclcpp::Node("tcp_pose_in_map"),
      executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>())
    {
        // ---------------- Parameters ----------------
        declare_parameter<std::string>("topic_tcp_in_base", "/manipulator/tcp_pose");
        declare_parameter<std::string>("topic_base_in_map", "/mobile_manipulator/base_pose");
        declare_parameter<std::string>("topic_tcp_in_map",  "/mobile_manipulator/tcp_pose");
        declare_parameter<std::string>("map_frame",         "map");
        declare_parameter<std::string>("mobile_base_frame", "mobile_base");
        declare_parameter<std::string>("base_link_frame",   "base_link");
        declare_parameter<double>("rate_hz", 500.0);

        topic_tcp_in_base_ = get_parameter("topic_tcp_in_base").as_string();
        topic_base_in_map_ = get_parameter("topic_base_in_map").as_string();
        topic_tcp_in_map_  = get_parameter("topic_tcp_in_map").as_string();
        map_frame_         = get_parameter("map_frame").as_string();
        mobile_base_frame_ = get_parameter("mobile_base_frame").as_string();
        base_link_frame_   = get_parameter("base_link_frame").as_string();

        loop_rate_hz_ = get_parameter("rate_hz").as_double();
        if (loop_rate_hz_ <= 0.0) { loop_rate_hz_ = 500.0; }
        dt_ = 1.0 / loop_rate_hz_;

        // ---------------- TF Buffer/Listener ----------------
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ---------------- Publisher ----------------
        pub_tcp_in_map_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_tcp_in_map_, 1);

        // ---------------- Subscribers (queue=1, custom options & CB groups) ----------------
        rclcpp::SubscriptionOptions sub_options;
        auto cb_group_sub_tcp = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_options.callback_group = cb_group_sub_tcp;

        sub_tcp_in_base_ = this->create_subscription<geometry_msgs::msg::Pose>(
            topic_tcp_in_base_, 1,
            std::bind(&WbqpControllerNode::tcpInBaseCb, this, std::placeholders::_1),
            sub_options);

        auto cb_group_sub_base = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_options.callback_group = cb_group_sub_base;

        sub_base_in_map_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_base_in_map_, 1,
            std::bind(&WbqpControllerNode::baseInMapCb, this, std::placeholders::_1),
            sub_options);

        // ---------------- Shutdown handler ----------------
        rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
            std::bind(&WbqpControllerNode::shutdown_handler, this)
        );

        // ---------------- One-time TF fetch: mobile_base -> base_link ----------------
        fetch_static_mobilebase_to_baselink();

        RCLCPP_INFO(get_logger(), "Node ready. Publishing %s as PoseStamped in %s at %.1f Hz",
                    topic_tcp_in_map_.c_str(), map_frame_.c_str(), loop_rate_hz_);
    }

    // ------------------------------ SPINNER ----------------------------- //
    void spinner()
    {
        // Add the node to the executor
        executor_->add_node(this->get_node_base_interface());

        // Create a steady clock to measure time
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);

        // Create timer callback with specified frequency
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            [this, &steady_clock]()
            {
                auto start_time = steady_clock.now();

                loopStep();

                double elapsed_time = (steady_clock.now() - start_time).seconds();
                spinner_mean_ = (spinner_mean_ * static_cast<double>(num_samples_) + elapsed_time)
                                / static_cast<double>(num_samples_ + 1);
                num_samples_++;
            });

        // Start spinning
        executor_->spin();
        // Shutdown the executor
        rclcpp::shutdown();
    }

    void shutdown_handler()
    {
        RCLCPP_INFO(get_logger(), "Shutting down. Mean loop time: %f s (samples: %zu)",
                    spinner_mean_, num_samples_);
    }

private:
    // ------------ Callbacks ------------
    void tcpInBaseCb(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        latest_tcp_in_base_ = *msg;
        have_tcp_in_base_ = true;
    }

    void baseInMapCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        latest_base_in_map_ = *msg;
        have_base_in_map_ = true;
    }

    // ------------ Main loop step ------------
    void loopStep()
    {
        // Need: base_in_map, tcp_in_base, and the stored mobile_base->base_link transform
        std::optional<geometry_msgs::msg::PoseStamped> base_in_map_opt;
        std::optional<geometry_msgs::msg::Pose>        tcp_in_base_opt;

        // Get latest data
        if (!have_base_in_map_ || !have_tcp_in_base_ || !have_mobilebase_to_baselink_) {
            return;
        }
        base_in_map_opt = latest_base_in_map_;
        tcp_in_base_opt = latest_tcp_in_base_;

        // Compose: T_map_tcp = T_map_mobile * T_mobile_base_base_link * T_base_link_tcp
        // Convert Pose/PoseStamped to tf2::Transform for composition.

        tf2::Transform T_map_mobile, T_base_link_tcp;

        // map -> mobile_base
        const auto & pm = base_in_map_opt->pose;
        tf2::fromMsg(pm, T_map_mobile);        

        // base_link -> tcp
        tf2::fromMsg(*tcp_in_base_opt, T_base_link_tcp);

        // Full transform
        tf2::Transform T_map_tcp = T_map_mobile * T_mobile_base_base_link_ * T_base_link_tcp;

        // Publish as PoseStamped
        geometry_msgs::msg::PoseStamped out;
        out.header.stamp    = base_in_map_opt->header.stamp;  // keep same time basis as base pose
        out.header.frame_id = map_frame_;
        
        const tf2::Vector3& t    = T_map_tcp.getOrigin();
        const tf2::Quaternion& q = T_map_tcp.getRotation();

        out.pose.position.x  = t.x();
        out.pose.position.y  = t.y();
        out.pose.position.z  = t.z();
        out.pose.orientation = tf2::toMsg(q);

        pub_tcp_in_map_->publish(out);
    }

    // ------------ One-time TF fetch ------------
    void fetch_static_mobilebase_to_baselink()
    {
        // Wait up to a few seconds for TF to be available
        const rclcpp::Time now = this->get_clock()->now();
        const rclcpp::Duration timeout(5s);

        RCLCPP_INFO(get_logger(), "Waiting for static TF %s -> %s ...",
                    mobile_base_frame_.c_str(), base_link_frame_.c_str());

        bool ok = tf_buffer_->canTransform(
            mobile_base_frame_, base_link_frame_,
            tf2::TimePointZero, tf2::durationFromSec(timeout.seconds()));

        if (!ok) {
            RCLCPP_WARN(get_logger(),
                        "Static TF %s -> %s not immediately available, trying a blocking lookup...",
                        mobile_base_frame_.c_str(), base_link_frame_.c_str());
        }

        try {
            auto tf_msg = tf_buffer_->lookupTransform(mobile_base_frame_, base_link_frame_, tf2::TimePointZero, 5s);

            tf2::fromMsg(tf_msg.transform, T_mobile_base_base_link_);
            have_mobilebase_to_baselink_ = true;

            RCLCPP_INFO(get_logger(), "Stored static TF %s -> %s.",
                        mobile_base_frame_.c_str(), base_link_frame_.c_str());

        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "Failed to get static TF %s -> %s: %s",
                         mobile_base_frame_.c_str(), base_link_frame_.c_str(), ex.what());
        }
    }

private:
    // Topics / frames
    std::string topic_tcp_in_base_;
    std::string topic_base_in_map_;
    std::string topic_tcp_in_map_;
    std::string map_frame_;
    std::string mobile_base_frame_;
    std::string base_link_frame_;

    // Publisher / Subscribers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tcp_in_map_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr     sub_tcp_in_base_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_base_in_map_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_mobilebase_to_baselink_{false};
    tf2::Transform T_mobile_base_base_link_;

    // Latest data
    geometry_msgs::msg::Pose        latest_tcp_in_base_;
    geometry_msgs::msg::PoseStamped latest_base_in_map_;
    bool have_tcp_in_base_{false};
    bool have_base_in_map_{false};

    // Timer / executor
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

    // Timing stats
    double dt_{0.002};
    double loop_rate_hz_{500.0};
    double spinner_mean_{0.0};
    size_t num_samples_{0};
};

// -------------------------- MAIN FUNCTION -------------------------- //
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WbqpControllerNode>();
    RCLCPP_INFO(node->get_logger(), "TCP Pose in Map node initialized successfully.");

    node->spinner();
    return 0;
}
