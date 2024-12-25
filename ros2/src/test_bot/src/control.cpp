// For moving robot from one point to other(Base)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

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
    RobotController() : Node("robot_controller"), duration_(1.0)
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

        // while ((elapsed_time <= duration_) and (distance[2] > 0.1))
        // while ((distance[2] > 0.1))
        while (elapsed_time < duration_)
        {
            rclcpp::spin_some(obj2);        // Updating robot position

            // std::cout<<elapsed_time.seconds()<<std::endl<<distance[2]<<std::endl<<obj2->coord.first<<std::endl<<obj2->coord.second<<std::endl<<distance[0]<<std::endl<<distance[1]<<std::endl<<std::endl<<std::endl;

            auto msg = geometry_msgs::msg::TwistStamped();
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "base_link";
            
            int p_val_x = 7.0;
            int p_val_y = 7.5;

            // msg.twist.linear.x = 0.74 * (distance[0] * p_val); // * 5.0;
            // msg.twist.linear.y = 1.0 * (distance[1] * p_val); // * 6.75;
            // msg.twist.linear.x = distance[0]; // * 5.0;
            // msg.twist.linear.y = distance[1]; // * 6.75;

            msg.twist.linear.x = ((distance[0]/(duration_ - elapsed_time)) * p_val_x); 
            msg.twist.linear.y = ((distance[1]/(duration_ - elapsed_time)) * p_val_y); 

            //msg.twist.linear.x = (distance[0] * p_val); 
            //msg.twist.linear.y = (distance[1] * p_val); 

            publisher_->publish(msg);


            // Logging
            std::cout<<"Bot Pose: "<<obj2->coord.first<<" "<<obj2->coord.second<<std::endl; 
            std::cout<<"Velocity: "<<msg.twist.linear.x<<" "<< msg.twist.linear.y<<std::endl; 
            std::cout<<"Remaining Distance: "<<distance[0]<<" "<<distance[1]<<" "<<distance[2]<<std::endl; 
            std::cout<<"Duration - Elapsed: "<<duration_-elapsed_time<<std::endl; 
            std::cout<<"Remaining Time: "<<duration_ - elapsed_time<<std::endl<<std::endl<<std::endl; 

            elapsed_time = this->get_clock()->now().seconds() - start_time_.seconds();
            distance = dist_speed(obj2->coord);

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
