// For moving robot from one point to other(PID Controller)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

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

            kp = 9.0;
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


class GetPose : public rclcpp::Node
{
public:
    GetPose() : Node("get_pose")
    {
        subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 10, std::bind(&GetPose::update, this, std::placeholders::_1));
    }

    std::pair<double, double> coord;  // For x and y coordinate

private:
    void update(const gazebo_msgs::msg::ModelStates & msg)
        {
            std::pair<bool, double> p1 = find_val(msg.name, "nice_bot");

            if (p1.first == true)
            {
                coord.first = msg.pose[p1.second].position.x;
                coord.second = msg.pose[p1.second].position.y;
            }
            // std::cout<<p1.first<<std::endl;
            // std::cout<<p1.second<<std::endl;
        }
    
    std::pair<bool, double> find_val(const std::vector<std::string>&  v1, std::string s1)
    {
        for (size_t i=0; i<v1.size(); i++)
        {
            if (v1[i] == s1)
            {
                return std::make_pair(true, i);
            }
        }
        return std::make_pair(false, 999);
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscriber_;
    
};

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller"), duration_(2.0)
    {                
        start_time_ = this->get_clock()->now();
        goal = std::make_pair(1.0, 1.0);
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mec_cont/reference", 10);

        publish_velocity();
    }

private:
    

    void publish_velocity()
    {

        elapsed_time = this->get_clock()->now().seconds() - start_time_.seconds();

        auto obj2 = std::make_shared<GetPose>();     
        rclcpp::spin_some(obj2);        // Updating robot position

        std::vector<double> distance = dist_speed(obj2->coord);      // Format:  vel_x, vel_y, dist 

        // PID Object
        PID pid_1 = PID();

        double rec_time = this->get_clock()->now().seconds();

        // while ((elapsed_time <= duration_) and (distance[2] > 0.1))
        // while ((distance[2] > 0.1))
        while (elapsed_time < duration_)
        {
            rclcpp::spin_some(obj2);       

            auto msg = geometry_msgs::msg::TwistStamped();
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "base_link";

            std::pair<double, double> output = pid_1.process(distance, this->get_clock()->now().seconds() - rec_time);    

            msg.twist.linear.x = output.first;
            msg.twist.linear.y = output.second;
            
            publisher_->publish(msg);


            // Logging
            std::cout<<"Bot Pose: "<<obj2->coord.first<<" "<<obj2->coord.second<<std::endl; 
            // std::cout<<"Velocity: "<<msg.twist.linear.x<<" "<< msg.twist.linear.y<<std::endl; 
            std::cout<<"Velocity(PID Output): "<<output.first<<" "<<output.second<<std::endl; 
            std::cout<<"Remaining Distance: "<<distance[0]<<" "<<distance[1]<<" "<<distance[2]<<std::endl; 
            // std::cout<<"Duration - Elapsed: "<<duration_-elapsed_time<<std::endl; 
            std::cout<<"Remaining Time: "<<duration_ - elapsed_time<<std::endl<<std::endl<<std::endl; 

            elapsed_time = this->get_clock()->now().seconds() - start_time_.seconds();
            distance = dist_speed(obj2->coord);
            rec_time = this->get_clock()->now().seconds();

            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 30));  // Sleep for 1/30th of a second (~33 milliseconds)
        }
        return;
    }

    std::vector<double> dist_speed(std::pair<double, double> bot_pose)
    {
        std::vector<double> output(3);
        output[0] = goal.first - bot_pose.first; 
        output[1] = goal.second - bot_pose.second; 
        output[2] = std::sqrt(std::pow(output[0], 2) + std::pow(output[1], 2));

        return output;
    }

    rclcpp::Time start_time_;
    double duration_;  
    double elapsed_time;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    std::pair<double, double> goal;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto obj1 =RobotController();
    rclcpp::shutdown();
    return 0;
}