#include "keyboard_reader.h"
#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "piper_msgs/msg/pos_cmd.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class KeyboardArmControl : public rclcpp::Node {
public:
    KeyboardArmControl() : Node("keyboard_arm_control") {
        // 声明参数
        this->declare_parameter("control_mode", "joint");
        this->declare_parameter("max_linear_velocity", 0.1);
        this->declare_parameter("max_angular_velocity", 0.5);
        this->declare_parameter("joint_velocity", 0.3);
        this->declare_parameter("gripper_min", 0.0);
        this->declare_parameter("gripper_max", 0.08);
        
        // 获取参数
        control_mode_ = this->get_parameter("control_mode").as_string();
        auto max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
        auto max_angular_velocity = this->get_parameter("max_angular_velocity").as_double();
        joint_velocity_ = this->get_parameter("joint_velocity").as_double();
        min_gripper_ = this->get_parameter("gripper_min").as_double();
        max_gripper_ = this->get_parameter("gripper_max").as_double();
        this->declare_parameter("linear_step", max_linear_velocity);
        this->declare_parameter("angular_step", max_angular_velocity);
        linear_step_ = this->get_parameter("linear_step").as_double();
        angular_step_ = this->get_parameter("angular_step").as_double();
        if (min_gripper_ > max_gripper_) {
            std::swap(min_gripper_, max_gripper_);
        }
        
        // 创建发布者
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/servo_server/delta_joint_cmds", 10);
        enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_flag", 10);
        pos_cmd_pub_ = this->create_publisher<piper_msgs::msg::PosCmd>(
            "pos_cmd", 10);
        {
            std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
            pos_cmd_state_ = piper_msgs::msg::PosCmd();
            pos_cmd_state_.gripper = std::clamp(0.0, min_gripper_, max_gripper_);
            pos_cmd_state_.mode1 = 0;
            pos_cmd_state_.mode2 = 0;
        }
        end_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "end_pose", 10,
            std::bind(&KeyboardArmControl::endPoseCallback, this, std::placeholders::_1));
        
        // 初始化键盘监听
        keyboard_reader_ = std::make_unique<KeyboardReader>();
        setupKeyCallbacks();
        
        // 打印控制说明
        printHelp();
        
        RCLCPP_INFO(this->get_logger(), "键盘机械臂控制节点已启动");
        RCLCPP_INFO(this->get_logger(), "控制模式: %s", control_mode_.c_str());
    }
    
    void start() {
        keyboard_reader_->startListening();
    }
    
    void stop() {
        keyboard_reader_->stopListening();
    }

private:
    void setupKeyCallbacks() {
        // 模式切换
        keyboard_reader_->registerKeyCallback('m', [this]() { toggleControlMode(); });
        keyboard_reader_->registerKeyCallback('h', [this]() { printHelp(); });
        
        // 使能控制
        keyboard_reader_->registerKeyCallback('e', [this]() { enableArm(); });
        keyboard_reader_->registerKeyCallback('c', [this]() { disableArm(); });
        
        // 笛卡尔空间控制（末端控制）
        keyboard_reader_->registerKeyCallback('w', [this]() { moveLinearX(1); });   // 前
        keyboard_reader_->registerKeyCallback('s', [this]() { moveLinearX(-1); });  // 后
        keyboard_reader_->registerKeyCallback('a', [this]() { moveLinearY(1); });   // 左
        keyboard_reader_->registerKeyCallback('d', [this]() { moveLinearY(-1); });  // 右
        keyboard_reader_->registerKeyCallback('q', [this]() { moveLinearZ(1); });   // 上
        keyboard_reader_->registerKeyCallback('z', [this]() { moveLinearZ(-1); });  // 下
        
        // 旋转控制
        keyboard_reader_->registerKeyCallback('i', [this]() { rotateX(1); });
        keyboard_reader_->registerKeyCallback('k', [this]() { rotateX(-1); });
        keyboard_reader_->registerKeyCallback('j', [this]() { rotateY(1); });
        keyboard_reader_->registerKeyCallback('l', [this]() { rotateY(-1); });
        keyboard_reader_->registerKeyCallback('u', [this]() { rotateZ(1); });
        keyboard_reader_->registerKeyCallback('o', [this]() { rotateZ(-1); });
        
        // 关节空间控制
        for (int i = 1; i <= 6; ++i) {
            keyboard_reader_->registerKeyCallback('0' + i, [this, i]() { moveJoint(i, 1); });
            keyboard_reader_->registerKeyCallback('0' + i + 6, [this, i]() { moveJoint(i, -1); });
        }
        
        // 夹爪控制
        keyboard_reader_->registerKeyCallback('g', [this]() { controlGripper(0.01); });
        keyboard_reader_->registerKeyCallback('b', [this]() { controlGripper(-0.01); });
        
        // 退出
        keyboard_reader_->registerKeyCallback(27, [this]() {  // ESC键
            RCLCPP_INFO(this->get_logger(), "正在关闭节点...");
            rclcpp::shutdown();
        });
    }
    
    void toggleControlMode() {
        if (control_mode_ == "joint") {
            control_mode_ = "cartesian";
            RCLCPP_INFO(this->get_logger(), "切换到笛卡尔空间控制模式");
        } else {
            control_mode_ = "joint";
            RCLCPP_INFO(this->get_logger(), "切换到关节空间控制模式");
        }
    }
    
    void moveLinearX(int direction) {
        if (control_mode_ != "cartesian") return;
        
        modifyAndPublish([&](piper_msgs::msg::PosCmd& cmd) {
            cmd.x += direction * linear_step_;
        });
    }
    
    void moveLinearY(int direction) {
        if (control_mode_ != "cartesian") return;
        
        modifyAndPublish([&](piper_msgs::msg::PosCmd& cmd) {
            cmd.y += direction * linear_step_;
        });
    }
    
    void moveLinearZ(int direction) {
        if (control_mode_ != "cartesian") return;
        
        modifyAndPublish([&](piper_msgs::msg::PosCmd& cmd) {
            cmd.z += direction * linear_step_;
        });
    }
    
    void rotateX(int direction) {
        if (control_mode_ != "cartesian") return;
        
        modifyAndPublish([&](piper_msgs::msg::PosCmd& cmd) {
            cmd.roll += direction * angular_step_;
        });
    }
    
    void rotateY(int direction) {
        if (control_mode_ != "cartesian") return;
        
        modifyAndPublish([&](piper_msgs::msg::PosCmd& cmd) {
            cmd.pitch += direction * angular_step_;
        });
    }
    
    void rotateZ(int direction) {
        if (control_mode_ != "cartesian") return;
        
        modifyAndPublish([&](piper_msgs::msg::PosCmd& cmd) {
            cmd.yaw += direction * angular_step_;
        });
    }
    
    void moveJoint(int joint_num, int direction) {
        if (control_mode_ != "joint") return;
        
        auto msg = std::make_unique<control_msgs::msg::JointJog>();
        msg->header.stamp = this->now();
        msg->joint_names.push_back("joint" + std::to_string(joint_num));
        msg->velocities.push_back(direction * joint_velocity_);
        joint_pub_->publish(std::move(msg));
    }
    
    void controlGripper(double delta) {
        modifyAndPublish([&](piper_msgs::msg::PosCmd& cmd) {
            cmd.gripper = std::clamp(cmd.gripper + delta, min_gripper_, max_gripper_);
        });
    }
    
    void publishPosCmd() {
        piper_msgs::msg::PosCmd cmd_copy;
        {
            std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
            cmd_copy = pos_cmd_state_;
        }
        pos_cmd_pub_->publish(cmd_copy);
    }

    void modifyAndPublish(const std::function<void(piper_msgs::msg::PosCmd&)>& modifier) {
        piper_msgs::msg::PosCmd cmd_copy;
        {
            std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
            modifier(pos_cmd_state_);
            cmd_copy = pos_cmd_state_;
        }
        pos_cmd_pub_->publish(cmd_copy);
    }

    void endPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
        pos_cmd_state_.x = msg->position.x;
        pos_cmd_state_.y = msg->position.y;
        pos_cmd_state_.z = msg->position.z;
        pos_cmd_state_.roll = roll;
        pos_cmd_state_.pitch = pitch;
        pos_cmd_state_.yaw = yaw;
    }
    
    void enableArm() {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        enable_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "机械臂使能");
    }
    
    void disableArm() {
        auto msg = std_msgs::msg::Bool();
        msg.data = false;
        enable_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "机械臂失能");
    }
    
    void printHelp() {
        std::cout << "\n=== 机械臂键盘控制帮助 ===\n";
        std::cout << "模式控制:\n";
        std::cout << "  m - 切换控制模式 (当前: " << control_mode_ << ")\n";
        std::cout << "  h - 显示此帮助信息\n";
        std::cout << "使能控制:\n";
        std::cout << "  e - 使能机械臂\n";
        std::cout << "  c - 失能机械臂\n";
        std::cout << "笛卡尔空间控制 (末端控制):\n";
        std::cout << "  平移步长: " << linear_step_ << " m, 旋转步长: " << angular_step_ << " rad\n";
        std::cout << "  w/s - 末端X轴方向移动\n";
        std::cout << "  a/d - 末端Y轴方向移动\n";
        std::cout << "  q/z - 末端Z轴方向移动\n";
        std::cout << "  i/k - 绕X轴旋转\n";
        std::cout << "  j/l - 绕Y轴旋转\n";
        std::cout << "  u/o - 绕Z轴旋转\n";
        std::cout << "关节空间控制:\n";
        std::cout << "  1-6 - 正向控制关节1-6\n";
        std::cout << "  7-0 - 反向控制关节1-6\n";
        std::cout << "夹爪控制:\n";
        std::cout << "  g - 打开夹爪\n";
        std::cout << "  b - 关闭夹爪\n";
        std::cout << "其他:\n";
        std::cout << "  ESC - 退出程序\n";
        std::cout << "==========================\n\n";
    }
    
    std::unique_ptr<KeyboardReader> keyboard_reader_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
    rclcpp::Publisher<piper_msgs::msg::PosCmd>::SharedPtr pos_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr end_pose_sub_;
    piper_msgs::msg::PosCmd pos_cmd_state_;
    std::mutex pos_cmd_mutex_;
    
    std::string control_mode_;
    double joint_velocity_;
    double min_gripper_;
    double max_gripper_;
    double linear_step_;
    double angular_step_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardArmControl>();
    
    // 启动键盘监听
    node->start();
    
    // 使用信号处理替代 atexit
    rclcpp::on_shutdown([node]() {
        node->stop();
    });
    
    rclcpp::spin(node);
    node->stop();
    
    rclcpp::shutdown();
    return 0;
}
