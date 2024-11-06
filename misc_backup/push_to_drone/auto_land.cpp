    /****************************************************************************
    *
    * Copyright 2020 PX4 Development Team. All rights reserved.
    *
    * Redistribution and use in source and binary forms, with or without
    * modification, are permitted provided that the following conditions are met:
    *
    * 1. Redistributions of source code must retain the above copyright notice, this
    * list of conditions and the following disclaimer.
    *
    * 2. Redistributions in binary form must reproduce the above copyright notice,
    * this list of conditions and the following disclaimer in the documentation
    * and/or other materials provided with the distribution.
    *
    * 3. Neither the name of the copyright holder nor the names of its contributors
    * may be used to endorse or promote products derived from this software without
    * specific prior written permission.
    *
    * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    * POSSIBILITY OF SUCH DAMAGE.
    *
    ****************************************************************************/


    /**
    * @brief Offboard control example
    * @file offboard_control.cpp
    * @addtogroup examples
    * @author Mickey Cowden <info@cowden.tech>
    * @author Nuno Marques <nuno.marques@dronesolutions.io>
    */


    #include <px4_msgs/msg/offboard_control_mode.hpp>
    #include <px4_msgs/msg/trajectory_setpoint.hpp>
    #include <px4_msgs/msg/vehicle_command.hpp>
    #include <px4_msgs/msg/vehicle_control_mode.hpp>
    #include <px4_msgs/msg/vehicle_local_position.hpp>
    #include <geometry_msgs/msg/transform_stamped.hpp>
    #include <geometry_msgs/msg/pose_stamped.hpp>
    #include <rclcpp/rclcpp.hpp>
    #include <stdint.h>
    #include <math.h>
    #include <chrono>
    #include <iostream>
    #include <string.h>


    #define FLIGHT_ALTITUDE         -1.75f                  // flight altitude (m)
    #define HOVER_ALT               -0.5f                   // hover altitude (m)
    #define RATE                    20                      // loop rate (hz)


    // hard-coded 
    #define PI  3.14159265358979323846264338327950


    using namespace std::chrono;
    using namespace std::chrono_literals;
    using namespace px4_msgs::msg;


    class OffboardControl : public rclcpp::Node
    {
    public:
    OffboardControl() : Node("offboard_control")
    {      
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);


        april_tag_detection_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tag_detections/tagpose_inertial", qos, [this](const geometry_msgs::msg::PoseStamped::UniquePtr msg){
            tag_pose_ = *msg;
        });

        init_tag_detection_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tag_detections/tagpose_body", qos, [this](const geometry_msgs::msg::PoseStamped::UniquePtr msg){
            init_pose_ = *msg;
        });



        local_pos_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg){
            current_position_ = *msg;
        });


        vehicle_command_listener_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos, [this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg){
            c_mode = *msg;
        });


        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {
            // offboard_control_mode needs to be paired with trajectory_setpoint
            double intpart;
            if(modf(((double)offboard_setpoint_counter_)/2, &intpart)==0.0){
                publish_offboard_control_mode();
            }
            publish_trajectory_setpoint();
        };
        timer_ = this->create_wall_timer(50ms, timer_callback);
    }




    private:
    rclcpp::TimerBase::SharedPtr timer_;


    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;


    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr april_tag_detection_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_tag_detection_subscriber_;
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_command_listener_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_subscriber_;


    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
        
    TrajectorySetpoint tag_trk_cmd_; // message for tracking tag

    geometry_msgs::msg::PoseStamped init_pose_;
    geometry_msgs::msg::PoseStamped tag_pose_;
    px4_msgs::msg::VehicleLocalPosition current_position_;
    VehicleControlMode c_mode;

    double pos_err[3] = {0,0,0}; // position error initialization
    double prev_pos_err[3] = {0,0,0}; // previous position error initialization
    double vel_err[3] = {0,0,0}; // velocity error initialization

    const double Kp = 0.48;
    const double Kd = 0.10;
    const double Kp_z =  0.1*Kp;
    const double Kd_z = 0.1*Kd;

    const double dt = 1.0/RATE;

    const std::string tag_des = "0"; // desired tag id
    double last_detected_tag_[3] = {0,0,0}; // initializing array for last-seen tag
    const double tag_tol = 0.0001; // less than a 0.1mm difference triggers update

    bool search_mode = true; // false means landing is engaged
                
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    };


    /**
    * @brief Publish the offboard control mode.
    *        For this example, only position and altitude controls are active.
    */
    void OffboardControl::publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.attitude = true;
        msg.body_rate = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }


    /**
    * @brief Publish a trajectory setpoint
    *        For this example, it sends a trajectory setpoint to make the
    *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
    */
    void OffboardControl::publish_trajectory_setpoint()
    {
        offboard_setpoint_counter_++;

        if(search_mode) {
            if(init_pose_.pose.position.z != 0) {
                last_detected_tag_[0] = tag_pose_.pose.position.x;
                last_detected_tag_[1] = tag_pose_.pose.position.y;
                last_detected_tag_[2] = tag_pose_.pose.position.z;
                printf("Tag found at inertial coordinates (%1.4f, %1.4f, %1.4f)\n",last_detected_tag_[0],last_detected_tag_[1],last_detected_tag_[2]);
                printf("Switching to landing mode!\n");
                search_mode = false;
            }
        }
        else {
            if (abs(tag_pose_.pose.position.x-last_detected_tag_[0]) >= tag_tol || abs(tag_pose_.pose.position.y-last_detected_tag_[1]) >= tag_tol || (tag_pose_.pose.position.z-last_detected_tag_[2]) >= tag_tol) {
                last_detected_tag_[0] = tag_pose_.pose.position.x;
                last_detected_tag_[1] = tag_pose_.pose.position.y;
                last_detected_tag_[2] = tag_pose_.pose.position.z;
                printf("Tag in sight!\n"); 
                printf("Inertial coordinates: (%1.4f, %1.4f, %1.4f)\n",last_detected_tag_[0],last_detected_tag_[1],last_detected_tag_[2]);
            }
            else {
                printf("Tag out of sight or estimate within tolerance. Proceeding with previous inertial estimate of tag position: (%1.4f, %1.4f, %1.4f)\n",last_detected_tag_[0],last_detected_tag_[1],last_detected_tag_[2]);
            }
            pos_err[0] = last_detected_tag_[0]-current_position_.x;
            pos_err[1] = last_detected_tag_[1]-current_position_.y;
            pos_err[2] = HOVER_ALT-current_position_.z;

            vel_err[0] = (pos_err[0]-prev_pos_err[0])/dt;
            vel_err[1] = (pos_err[1]-prev_pos_err[1])/dt;

            tag_trk_cmd_.velocity[0] = Kp*pos_err[0];
            tag_trk_cmd_.acceleration[0] = Kd*vel_err[0];
            tag_trk_cmd_.position[0] = current_position_.x+(tag_trk_cmd_.velocity[0]/dt);

            tag_trk_cmd_.velocity[1] = Kp*pos_err[1];
            tag_trk_cmd_.acceleration[1] = Kd*vel_err[1];
            tag_trk_cmd_.position[1] = current_position_.y+(tag_trk_cmd_.velocity[1]/dt);

            tag_trk_cmd_.velocity[2] = Kp_z*pos_err[2];
            tag_trk_cmd_.acceleration[2] = Kd_z*vel_err[2];
            tag_trk_cmd_.position[2] = current_position_.z+(tag_trk_cmd_.velocity[2]/dt);

            tag_trk_cmd_.timestamp = this->get_clock()->now().nanoseconds()/1000;

            trajectory_setpoint_publisher_->publish(tag_trk_cmd_);

            prev_pos_err[0] = pos_err[0];
            prev_pos_err[1] = pos_err[1];
            prev_pos_err[2] = pos_err[2];
        }


    }

    int main(int argc, char *argv[])
    {
        std::cout << "Starting offboard control node..." << std::endl;
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<OffboardControl>());


        rclcpp::shutdown();
        return 0;
    }







