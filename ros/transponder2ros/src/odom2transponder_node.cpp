
// Example node of publishing a transponder message from an odom message

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <optional>

#include <GeographicLib/LocalCartesian.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "transponder_msgs/msg/transponder.hpp"

class Odom2Transponder : public rclcpp::Node
{
public:
    Odom2Transponder()
        : Node("odom2transponder")
    {

        RCLCPP_INFO(this->get_logger(), "Odomentry-to-Transponder Node");

        // Parameters
        this->declare_parameter("odometry_in", "/state/odom");
        this->declare_parameter("carmode_in", "/trajectory/mode/requested");
        this->declare_parameter("transponder_out", "/transponder/out");
        std::string param_odometryIn = this->get_parameter("odometry_in").as_string();
        std::string param_carModeIn = this->get_parameter("carmode_in").as_string();
        std::string param_transponderOut = this->get_parameter("transponder_out").as_string();

        this->declare_parameter<double>("lat0", 0.0);
        this->declare_parameter<double>("lon0", 0.0);
        this->declare_parameter<double>("alt0", 0.0);
        lla0_.latitude = this->get_parameter("lat0").as_double();
        lla0_.longitude = this->get_parameter("lon0").as_double();
        lla0_.altitude = this->get_parameter("alt0").as_double();

        this->declare_parameter<int>("car", 1);  // Car number
        car_id_ = this->get_parameter("car").as_int();

        frame_id_ = this->declare_parameter("transform_to", "");

        if (!frame_id_.empty()) {
            tf_buffer_.emplace(this->get_clock());
            tf_listener_.emplace(*tf_buffer_);
        }

        // Publishers
        pub_Transponder_ = this->create_publisher<transponder_msgs::msg::Transponder>(param_transponderOut, 1);

        // Subscribers
        sub_Odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            param_odometryIn,
            1,
            std::bind(
                &Odom2Transponder::callback_Odometry,
                this, std::placeholders::_1)
        );

        /*
        // For those that want to fill in the 'state' field, here's an example of how to do it
        // Each different team will have different modes, so this is just an example for CAST Racer
        sub_CarMode_ = this->create_subscription<iac_msgs::msg::CarMode>(
            param_carModeIn,
            1,
            std::bind(
                &Odom2Transponder::callback_CarMode,
                this, std::placeholders::_1)
        );
        */

        // Timers
        start_timer_aligned(100);
    }

private:
    double start_timer_aligned(int period_ms)
    {
        // Starts a timer aligned with the second boundary with period `period_ms`
        // Note: period_ms should evenly fit into a second for this to work
        double period = static_cast<double>(period_ms) / 1000.0;
        double t_now = this->get_clock()->now().seconds();
        double delay = period - std::fmod(t_now, period);

        RCLCPP_INFO(this->get_logger(), "Delaying %.3f s to align with next period", delay);
        std::this_thread::sleep_for(std::chrono::duration<double>(delay));

        timer_pushTransponder_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),  
            std::bind(&Odom2Transponder::callback_pushTransponder, this));
    }
    void callback_pushTransponder()
    {
        // Check timestamp is reasonable
        rclcpp::Duration t_late_by = this->get_clock()->now() - odom_.header.stamp;
        if (abs(t_late_by.seconds()) > 1.0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5UL * 1000 * 1000, "Odom late by %.1f",t_late_by.seconds());
            // RCLCPP_WARN(this->get_logger(), "Odom late by %.1f",t_late_by.seconds());
            return;
        }

        // transform to rear axle if required
        if (tf_buffer_) 
        {
            odom_ = transform_Odometry(odom_);
        }

        // Convert enu to lla
        geographic_msgs::msg::GeoPoint lla = enu_to_lla_geodetic(odom_.pose.pose.position, lla0_);

        // Calculate yaw
        geometry_msgs::msg::Quaternion q = odom_.pose.pose.orientation;
        tf2::Quaternion q_tf;
        tf2::fromMsg(q, q_tf);
        tf2::Matrix3x3 R(q_tf); // rotation matrix body->ENU

        tf2::Vector3 v_local(
            odom_.twist.twist.linear.x,
            odom_.twist.twist.linear.y,
            odom_.twist.twist.linear.z
        );

        tf2::Vector3 v_enu = R * v_local;

        // Push message
        transponder_msgs::msg::Transponder msg;

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        msg.car_id = car_id_;
        msg.lat = lla.latitude;
        msg.lon = lla.longitude;
        msg.alt = lla.altitude;
        msg.v_east = v_enu.x();
        msg.v_north = v_enu.y();
        msg.v_up = v_enu.z();
        msg.state = car_mode_; 

        pub_Transponder_->publish(msg);
    }

    void callback_Odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Save the latest odom data
        odom_ = *msg;

        // Done
        return;
    }

    nav_msgs::msg::Odometry transform_Odometry(const nav_msgs::msg::Odometry& msg) {
        // We assume that odom frame and to frame have the same orientation w.r.t robot
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(msg.child_frame_id, frame_id_, msg.header.stamp);

            nav_msgs::msg::Odometry transformed = msg;

            // pose
            const auto& trans = transform.transform.translation;
            tf2::Vector3 r_AB(trans.x, trans.y, trans.z);

            const auto& pose = transformed.pose.pose;
            tf2::Quaternion q;
            tf2::fromMsg(pose.orientation, q);
            tf2::Matrix3x3 R(q);

            tf2::Vector3 p_A(pose.position.x, pose.position.y, pose.position.z);
            tf2::Vector3 p_B = p_A + R * r_AB;

            transformed.pose.pose.position.x = p_B.x();
            transformed.pose.pose.position.y = p_B.y();
            transformed.pose.pose.position.z = p_B.z();

            // velocity
            const auto& lin = transformed.twist.twist.linear;
            const auto& ang = transformed.twist.twist.angular;

            tf2::Vector3 v_A(lin.x, lin.y, lin.z);
            tf2::Vector3 w_A(ang.x, ang.y, ang.z);

            tf2::Vector3 v_B = v_A + w_A.cross(r_AB);

            transformed.twist.twist.linear.x = v_B.x();
            transformed.twist.twist.linear.y = v_B.y();
            transformed.twist.twist.linear.z = v_B.z();

            transformed.child_frame_id = frame_id_;

            return transformed;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_STREAM(this->get_logger(), "TF Lookup failed: " << ex.what());
            return msg;
        }
    }

    /*
    // For those that want to fill in the 'state' field, here's an example of how to do it
    // Each different team will have different modes, so this is just an example for CAST Racer

    void callback_CarMode(const iac_msgs::msg::CarMode::SharedPtr msg)
    {
        // Save the latest odom data
        switch (msg->mode)
        {
            case (iac_msgs::msg::CarMode::TERMINATE) :
            case (iac_msgs::msg::CarMode::EMERGENCY_STOP) :
                car_mode_ = transponder_msgs::msg::Transponder::STATE_EMERGENCY_STOP;
                break;
            case (iac_msgs::msg::CarMode::CONTROLLED_STOP) :
                car_mode_ = transponder_msgs::msg::Transponder::STATE_CONTROLLED_STOP;
                break;
            case (iac_msgs::msg::CarMode::ENGINE_IDLE) :
            case (iac_msgs::msg::CarMode::PIT) :
            case (iac_msgs::msg::CarMode::ADAPTIVE_CRUISE) :
            case (iac_msgs::msg::CarMode::OVERTAKE_ALLOWED) :
            case (iac_msgs::msg::CarMode::DEFENDER) :
            case (iac_msgs::msg::CarMode::RACE) :
                car_mode_ = transponder_msgs::msg::Transponder::STATE_NOMINAL;
                break;
            case (iac_msgs::msg::CarMode::UNKNOWN) :
            default:
                car_mode_ = transponder_msgs::msg::Transponder::STATE_UNKNOWN;
                break;
        }

        // Done
        return;
    }
    */

    geographic_msgs::msg::GeoPoint enu_to_lla_geodetic(
        const geometry_msgs::msg::Point enu,
        const geographic_msgs::msg::GeoPoint lla0)
    {
        static GeographicLib::LocalCartesian local_cartesian(
            lla0.latitude, lla0.longitude, lla0.altitude,
            GeographicLib::Geocentric::WGS84()
        );

        // Convert local Cartesian coordinates to geodetic coordinates
        geographic_msgs::msg::GeoPoint out;
        local_cartesian.Reverse(
            enu.x, enu.y, enu.z,
            out.latitude, out.longitude, out.altitude
        );

        return out;
    }

    // Publishers / Subscribers / Timers
    rclcpp::Publisher<transponder_msgs::msg::Transponder>::SharedPtr pub_Transponder_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_Odometry_;
    // rclcpp::Subscription<iac_msgs::msg::CarMode>::SharedPtr sub_CarMode_;  // Needed to fill in the 'state' field
    rclcpp::TimerBase::SharedPtr timer_pushTransponder_;

    // TF, only enabled if transform requested
    std::optional<tf2_ros::Buffer> tf_buffer_;
    std::optional<tf2_ros::TransformListener> tf_listener_;

    // Variables
    std::string frame_id_;
    uint8_t car_id_;
    nav_msgs::msg::Odometry odom_;
    geographic_msgs::msg::GeoPoint lla0_;
    uint8_t car_mode_ = transponder_msgs::msg::Transponder::STATE_UNKNOWN;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odom2Transponder>());
    rclcpp::shutdown();
    return 0;
}
