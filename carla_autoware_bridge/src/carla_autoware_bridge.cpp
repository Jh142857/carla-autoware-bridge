//
// Created by cjh on 23-7-5.
//
// # include "rclcpp/rclcpp.hpp"
//
// class CarlaAutowareBridge: public rclcpp::Node
// {
//
// };

#include <memory>
#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>

#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <std_msgs/msg/float32.hpp>


class CarlaAutowareBridge: public rclcpp::Node
{
  public:
    CarlaAutowareBridge();
    ~CarlaAutowareBridge();
  private:
    using State = localization_interface::InitializationState;
    void change_state(const State::Message::_state_type& state);
    void on_odometry(const nav_msgs::msg::Odometry& msg);
    void on_vehicle_status(const carla_msgs::msg::CarlaEgoVehicleStatus& msg);
    void on_control_cmd(const autoware_auto_control_msgs::msg::AckermannControlCommand& msg);
    // void on_speed(const std_msgs::msg::Float32& msg);
    void on_predicted_objects(const derived_object_msgs::msg::ObjectArray& msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<State::Message>::SharedPtr pub_state_;
    // component_interface_utils::Publisher<State>::SharedPtr pub_state_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematics_;

    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr sub_vehicle_status_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_vel_report_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr pub_gear_report_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steer_report_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr pub_control_mode_report_;
    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_accel_;

    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_control_cmd_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr pub_vehicle_control_;
    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;

    // perception
    rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr sub_predicted_objects_;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr pub_predicted_objects_;

    State::Message state_;
    rclcpp::Time time_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool is_initialized_;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
    double speed_;
};

CarlaAutowareBridge::CarlaAutowareBridge(): Node("carla_pose_initializer")
{
  is_initialized_ = false;
  speed_ = 0.0;
  // const auto node = component_interface_utils::NodeAdaptor(this);
  // RCLCPP_WARN(this->get_logger(), localization_interface::InitializationState::name);
  // node.init_pub(pub_state_);

  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 1000,
     std::bind(&CarlaAutowareBridge::on_odometry, this, std::placeholders::_1));
  // 发布里程信息
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  pub_kinematics_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", qos);
  pub_init_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose3d", qos);
  pub_state_ = this->create_publisher<State::Message>("/api/localization/initialization_state", qos);
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose_twist_fusion_filter/pose", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // autoware cmd msg to carla
  sub_control_cmd_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", 1,
                                                                                                           std::bind(&CarlaAutowareBridge::on_control_cmd, this, std::placeholders::_1));
  pub_vehicle_control_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1);
  sub_vehicle_status_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>("/carla/ego_vehicle/vehicle_status", 10,
     std::bind(&CarlaAutowareBridge::on_vehicle_status, this, std::placeholders::_1));
  // 发布车辆状态信息
  pub_vel_report_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 1);
  pub_gear_report_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 1);
  pub_steer_report_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 1);
  pub_control_mode_report_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 1);
  pub_accel_ = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("/localization/acceleration", 1);
  // speed
  // sub_speed_ = this->create_subscription<std_msgs::msg::Float32>("/carla/ego_vehicle/speedometer", 1,
  //            std::bind(&CarlaAutowareBridge::on_speed, this, std::placeholders::_1));

  // perception
  sub_predicted_objects_ = this->create_subscription<derived_object_msgs::msg::ObjectArray>("/carla/ego_vehicle/objects", 1,
         std::bind(&CarlaAutowareBridge::on_predicted_objects, this, std::placeholders::_1));
  pub_predicted_objects_ = this->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("/perception/object_recognition/objects", 1);
}

CarlaAutowareBridge::~CarlaAutowareBridge()
{
  // change_state(State::Message::UNINITIALIZED);
}

void CarlaAutowareBridge::change_state(const State::Message::_state_type& state)
{
    state_.stamp = get_clock()->now();
    state_.state = state;
    pub_state_->publish(state_);
}

void CarlaAutowareBridge::on_odometry(const nav_msgs::msg::Odometry& msg)
{
    // RCLCPP_WARN(this->get_logger(), "on_odometry");
    if (!is_initialized_)
    {
        // 保存初始位置
        initial_pose_.header = msg.header;
        initial_pose_.pose = msg.pose;
        is_initialized_ = true;
    }
    // 发布初始位置，header为当前时间
    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.header = msg.header;
    init_pose.header.stamp = get_clock()->now();
    init_pose.pose = initial_pose_.pose;
    pub_init_pose_->publish(init_pose);

    // 发布定位位置，header为当前时间
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.header.stamp = get_clock()->now();
    pose.pose = msg.pose.pose;
    pub_pose_->publish(pose);

    // 发布tf，map->base_link
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg.header;
    // 这个时间与lookupTransform的时间需要相同
    transform.header.stamp = get_clock()->now();
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = msg.pose.pose.position.x;
    transform.transform.translation.y = msg.pose.pose.position.y;
    transform.transform.translation.z = msg.pose.pose.position.z;
    transform.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    // RCLCPP_INFO(get_logger(), "time: %f", msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);
    // 里程计信息
    nav_msgs::msg::Odometry odom;
    odom.header = msg.header;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.header.stamp = get_clock()->now();
    // RCLCPP_INFO(get_logger(), "time1: %f, time2: %f", get_clock()->now().seconds() + get_clock()->now().nanoseconds() * 1e-9,
    //             now().seconds() + now().nanoseconds() * 1e-9);
    odom.pose = msg.pose;
    odom.twist = msg.twist;
    pub_kinematics_->publish(odom);

    // 改变状态，初始化完成
    change_state(State::Message::INITIALIZED);
    // std::cout << State::Message::INITIALIZED << std::endl;
}

void CarlaAutowareBridge::on_vehicle_status(const carla_msgs::msg::CarlaEgoVehicleStatus &msg) {
    // 发布车辆状态信息
    autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report;
    velocity_report.header.stamp = get_clock()->now();
    velocity_report.header.frame_id = "base_link";
    velocity_report.longitudinal_velocity = msg.velocity;
    // velocity_report.lateral_velocity = 0.0;
    velocity_report.heading_rate = 0.0;
    pub_vel_report_->publish(velocity_report);
    // RCLCPP_INFO(get_logger(), "velocity: %f", msg.velocity);

    // 保存速度信息
    speed_ = msg.velocity;

    autoware_auto_vehicle_msgs::msg::GearReport gear_report;
    gear_report.stamp = get_clock()->now();
    gear_report.report = autoware_auto_vehicle_msgs::msg::GearReport::DRIVE;
    pub_gear_report_->publish(gear_report);

    autoware_auto_vehicle_msgs::msg::SteeringReport steer_report;
    steer_report.stamp = get_clock()->now();
    // time_ = steer_report.stamp;
    // TODO： check angle right or not
    steer_report.steering_tire_angle = -msg.control.steer;
    // RCLCPP_WARN(get_logger(), "steer: %f", steer_report.steering_tire_angle);
    pub_steer_report_->publish(steer_report);

    autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_report;
    control_mode_report.stamp = get_clock()->now();
    control_mode_report.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    pub_control_mode_report_->publish(control_mode_report);

    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.header.stamp = get_clock()->now();
    //
    // RCLCPP_INFO(get_logger(), "-------------------------------");
    // RCLCPP_INFO(get_logger(), "time1: %f", accel.header.stamp.sec + accel.header.stamp.nanosec * 1e-9);
    accel.header.frame_id = "base_link";
    accel.accel.accel = msg.acceleration;
    constexpr auto cov = 0.001;
    accel.accel.covariance.at(0) = cov;     // linear x
    accel.accel.covariance.at(7) = cov;     // linear y
    accel.accel.covariance.at(14) = cov;    // linear z
    accel.accel.covariance.at(21) = cov;    // angular x
    accel.accel.covariance.at(28) = cov;    // angular y
    accel.accel.covariance.at(35) = cov;    // angular z
    pub_accel_->publish(accel);
}

void CarlaAutowareBridge::on_control_cmd(const autoware_auto_control_msgs::msg::AckermannControlCommand &msg) {
    // 发布车辆控制信息
    carla_msgs::msg::CarlaEgoVehicleControl control;
    // control.header.stamp = msg.stamp;
    control.steer = -msg.lateral.steering_tire_angle;
    double speed_diff = msg.longitudinal.speed - speed_;
    // RCLCPP_INFO(this->get_logger(), "speed_diff: %f, speed: %f, msg.speed: %f", speed_diff, speed_, msg.longitudinal.speed);
    if (speed_diff > 0.0) {
        control.throttle = 0.75;
        control.brake = 0.0;
    } else {
        control.throttle = 0.0;
        if (msg.longitudinal.speed <= 0.0) {
            control.brake = 0.75;
        }
        else if (speed_diff > -1) {
            control.brake = 0.0;
        }
        else {
            control.brake = 0.01;
        }
    }
    pub_vehicle_control_->publish(control);
}

void CarlaAutowareBridge::on_predicted_objects(const derived_object_msgs::msg::ObjectArray &msg) {

    autoware_auto_perception_msgs::msg::PredictedObjects predicted_objects;
    predicted_objects.header = msg.header;
    predicted_objects.header.stamp = get_clock()->now();
    for (auto & obj : msg.objects) {
        autoware_auto_perception_msgs::msg::PredictedObject predicted_object;
        // classification
        autoware_auto_perception_msgs::msg::ObjectClassification classification;
        if (obj.classification == 6) {
            classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
            classification.probability = 1.0;
        }
        predicted_object.classification.push_back(classification);
        // kinematics
        autoware_auto_perception_msgs::msg::PredictedObjectKinematics kinematics;
        kinematics.initial_pose_with_covariance.pose = obj.pose;
        kinematics.initial_twist_with_covariance.twist = obj.twist;
        kinematics.initial_acceleration_with_covariance.accel = obj.accel;
        // predicted paths
        // autoware_auto_perception_msgs::msg::PredictedPath predicted_path;
        predicted_object.kinematics = kinematics;

        // shape
        autoware_auto_perception_msgs::msg::Shape shape;
        shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
        shape.footprint.points = obj.polygon.points;
        shape.dimensions.x = obj.shape.dimensions.at(0);
        shape.dimensions.y = obj.shape.dimensions.at(1);
        shape.dimensions.z = obj.shape.dimensions.at(2);
        predicted_object.shape = shape;

        predicted_objects.objects.push_back(predicted_object);
    }
    // pub_predicted_objects_->publish(predicted_objects);
}

// void CarlaAutowareBridge::on_speed(const std_msgs::msg::Float32 &msg) {
//     // speed_ = msg.data;
// }

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor executor;
    // auto node = std::make_shared<CarlaAutowareBridge>();
    // executor.add_node(node);
    // executor.spin();
    // executor.remove_node(node);
    rclcpp::spin(std::make_shared<CarlaAutowareBridge>());
    rclcpp::shutdown();
    return 0;
}