// To  receive robot position estimate from pico.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


#include <iostream>
#include <sstream>
#include <vector>

using std::placeholders::_1;


class PID
{   
    public:
        // std::pair<double,double> PID()
        PID()
        {
            error = std::pair(0.0, 0.0);
            prev_error = std::pair(0.0, 0.0);

            integral = std::pair(0.0, 0.0);
            derivative = std::pair(0.0, 0.0);

            kp = 10.0;
            ki = 0.0;
            kd = 0.0;
        }

        std::pair<double,double> process(std::vector<double> distance, double dt)
        {
            error.first = distance[0]; 
            error.second = distance[1];

            integral.first += error.first;
            integral.second += error.second;

            derivative.first = (error.first - prev_error.first) / dt;
            derivative.second = (error.second - prev_error.second) / dt;

            prev_error = error;

            return std::pair(error.first * kp + integral.first * ki + derivative.first * kd, error.second * kp + integral.second * ki + derivative.second * kd );
        }

    
    private:
        std::pair<double, double> error;
        std::pair<double, double> prev_error;

        std::pair<double, double> integral;
        std::pair<double, double> derivative;

        double kp;
        double ki;
        double kd;

};


class GoToGoal : public rclcpp::Node
{
    public:
        GoToGoal() : Node("go_to_goal")
        {
            subscription_pose_est_ = this->create_subscription<std_msgs::msg::String>("pose_estimate", 10, std::bind(
                &GoToGoal::topic_callback, this, _1));

            publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mec_cont/reference", 10);

            goal = std::make_pair(1,1);

            duration = 2.0;
            start_time_ = this->get_clock()->now();
            elapsed_time = this->get_clock()->now().seconds() - start_time_.seconds();

            pid_1 = PID();    

            rec_time = this->get_clock()->now().seconds();

        }


    private:
        void topic_callback(const std_msgs::msg::String & msg)
        {
            
            if (elapsed_time < duration){

                std::stringstream ss(msg.data);
                std::string part;
                std::vector<std::string> result;

                while (std::getline(ss, part, '@')) {
                    result.push_back(part);
                }

                distance = dist_speed(std::make_pair(std::stod(result[0]), std::stod(result[1])), goal);      // Distance from current pose to goal.

                if (distance[2]>2.0){
                    std::cout<<"Terminating. Cannot reach goal in time."<<std::endl;    
                }


                auto msg = geometry_msgs::msg::TwistStamped();
                msg.header.stamp = this->get_clock()->now();
                msg.header.frame_id = "base_link";

                std::pair<double, double> output = pid_1.process(distance, this->get_clock()->now().seconds() - rec_time);    

                msg.twist.linear.x = output.first;
                msg.twist.linear.y = output.second;

                // Publish to controller
                publisher_->publish(msg);

                elapsed_time = this->get_clock()->now().seconds() - start_time_.seconds();
                rec_time = this->get_clock()->now().seconds();                
            }
        }

        std::vector<double> dist_speed(std::pair<double, double> bot_pose, std::pair<double, double> goal)
            {
                std::vector<double> output(3);
                output[0] = goal.first - bot_pose.first; 
                output[1] = goal.second - bot_pose.second; 
                output[2] = std::sqrt(std::pow(output[0], 2) + std::pow(output[1], 2));
                
                //output[0] = output[0]/remaning_time;
                //output[1] = output[1]/remaning_time;

                return output;
            }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_pose_est_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;    

        std::pair<double, double> goal; // x, y  
        
        double duration;
        double elapsed_time;
        rclcpp::Time start_time_;
        double rec_time;


        std::vector<double> distance;   // Format:  vel_x, vel_y, dist 

        PID pid_1;

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToGoal>());
  rclcpp::shutdown();
  return 0;
}



// std::shared_ptr<rclcpp::Node> node 