#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <typeinfo>


// For getting the robot position
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


// For moving the cart to the goal
class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {                
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mec_cont/reference", 10);
    }
    
    void publish_velocity(std::pair<double, double> goal, double duration)
    {
        start_time_ = this->get_clock()->now();
        elapsed_time = this->get_clock()->now().seconds() - start_time_.seconds();

        auto obj2 = std::make_shared<GetPose>();        

        std::vector<double> distance = dist_speed(obj2->coord, goal, elapsed_time - duration);   // Format:  vel_x, vel_y, dist 

        auto msg = geometry_msgs::msg::TwistStamped();

        // while ((elapsed_time <= duration_) and (distance[2] > 0.1))
        while ((distance[2] > 0.5))
        {
            rclcpp::spin_some(obj2);        // Updating robot position

            // std::cout<<elapsed_time<<std::endl<<distance[2]<<std::endl<<obj2->coord.first<<std::endl<<obj2->coord.second<<std::endl<<distance[0]<<std::endl<<distance[1]<<std::endl<<std::endl<<std::endl;

            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "base_link";
            
            int p_val = 2.0;

            msg.twist.linear.x = 0.74 * (distance[0] * p_val); // * 5.0;
            msg.twist.linear.y = 1.0 * (distance[1] * p_val); // * 6.75;

            // msg.twist.linear.x = distance[0]; // * 5.0;
            // msg.twist.linear.y = distance[1]; // * 6.75;

            publisher_->publish(msg);

            elapsed_time = this->get_clock()->now().seconds() - start_time_.seconds();
            distance = dist_speed(obj2->coord, goal, elapsed_time - duration);
        }

        std::cout<<"Goal Reached!"<<std::endl;
            
        msg.twist.linear.x = distance[0] * -5.0;
        msg.twist.linear.y = distance[1] * -5.0;
        publisher_->publish(msg);

        return;
    }

private:
    std::vector<double> dist_speed(std::pair<double, double> bot_pose, std::pair<double, double> goal, double remaning_time)
    {
        std::vector<double> output(3);
        output[0] = goal.first - bot_pose.first; 
        output[1] = goal.second - bot_pose.second; 
        output[2] = std::sqrt(std::pow(output[0], 2) + std::pow(output[1], 2));
        
        //output[0] = output[0]/remaning_time;
        //output[1] = output[1]/remaning_time;

        return output;
    }

    rclcpp::Time start_time_;
    double elapsed_time;

    // std::pair<double, double> goal(1.0, 1.0);  
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;    
};


// For calculating Arm Inverse Kinematics(IK)
class IkSolver
{
    public:
        IkSolver(std::shared_ptr<rclcpp::Node> node) : node_(node)
        {
            RCLCPP_INFO(node_->get_logger(), "Inside Class 2");                
            subscription_ = node->create_subscription<std_msgs::msg::String>(
                "robot_description",
                rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
                std::bind(&IkSolver::robotDescriptionCallback, this, std::placeholders::_1));            
        }

        void robotDescriptionCallback(const std_msgs::msg::String& msg)
        {
            // Construct Tree from URDF
            const std::string urdf = msg.data;
            kdl_parser::treeFromString(urdf, tree_);            

            // Get Kinematic Chain
            tree_.getChain("base_link_x", "racquet_link", chain_);

            // Create IK Solver
            solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);        

            RCLCPP_INFO(node_->get_logger(), "Inside Solver...........");                    
        }

        KDL::JntArray get_desired_joint_angles(std::vector<double> init_joint, KDL::Frame goal_frame)    
        {
            // RCLCPP_INFO(node_->get_logger(), "Inside CartToJoint...........");                    
            // Initial Joint State 
            KDL::JntArray q_init(chain_.getNrOfJoints());

            // REFERENCE:

            // Before Sort:
            //     left_wheel_joint
            //     right_wheel_joint
            //     dummy_left_joint
            //     dummy_right_joint
            //     base_link_x_y
            //     base_link_z_1
            //     base_link_y_z
            //     l3_joint

            // After Sort:
            //     base_link_x_y
            //     base_link_y_z
            //     base_link_z_1
            //     dummy_left_joint
            //     dummy_right_joint
            //     l3_joint
            //     left_wheel_joint
            //     right_wheel_joint
            
            // std::exit(0);

            q_init(0) = init_joint[0];
            q_init(1) = init_joint[1];
            q_init(2) = init_joint[2];
            q_init(3) = init_joint[5];
            
            
            // Desired pose of end-effector 
            const KDL::Frame p_in = goal_frame;

            // To store output
            KDL::JntArray q_out(chain_.getNrOfJoints());

            // Run IK Solver
            solver_->CartToJnt(q_init, p_in, q_out);
            
            return q_out;
        }

    private:
        std::shared_ptr<rclcpp::Node> node_;
        
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
};









class IkArm : public rclcpp::Node
{
    public:
        IkArm() : Node("IkNode")
        {
            RCLCPP_INFO(this->get_logger(), "Initializaing Arm...........");                    
            
            //for(unsigned int i = 0; i < previous_.rows(); ++i) {
            //    previous_(i) = 0.0; // Initialize all elements to 0
            //}
        }
                
        std::shared_ptr<rclcpp::Node> init()
        {
            obj1_ = std::make_shared<IkSolver>(shared_from_this());            

            // To get current joint state.
            joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&IkArm::get_current_joint_angles, this, std::placeholders::_1)); 
                        
            // Set coordinate of cartisian space to reach.
            // goal_frame = KDL::Frame(KDL::Vector(0.5, 0.0, 1.0));
            goal_frame = KDL::Frame(KDL::Vector(0.0, 0.0, 0.0));
            goal_frame_cart = KDL::Frame(KDL::Vector(0.0, 0.0, 0.0));
            arm_move_time = 0.0;

            current_frame = KDL::Frame(KDL::Vector(0.0, 0.0, 0.0));

            // To set the desired joint angles for Joint_Trajectory_Controller
            publish_state = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_position_controller/joint_trajectory", 10);
            
            // To get coordinate where to hit the ball
            ball_coord = this->create_subscription<std_msgs::msg::String>(
                "ball_trajectory", 10, std::bind(&IkArm::process_data, this, std::placeholders::_1));

            return shared_from_this();
        }

        void get_current_joint_angles(const sensor_msgs::msg::JointState & msg)
        {
            joint_names_ = msg.name;
            joint_position_ = msg.position;

            /*            
            std::stringstream ss;

            for (const auto & name : joint_names_)
            {
                ss << name << " ";
            }

            RCLCPP_INFO(this->get_logger(), "Joint Names: %s", ss.str().c_str());

            std::stringstream ss2;

            for (const auto & i : joint_position_)
            {
                ss2 << std::to_string(i) << " ";
                //RCLCPP_INFO(this->get_logger(), "Joint Position: %f", i);
            }
            RCLCPP_INFO(this->get_logger(), "Joint Positions: %s", ss2.str().c_str());
            */
            
            

            // To sort `joint_names_` and `joint_position_`.

            // Creating Temp pair
            std::vector<std::pair<std::string, double>> temp_pair;
            for (size_t i=0; i<joint_names_.size(); ++i)
            {
                temp_pair.push_back(std::make_pair(joint_names_[i], joint_position_[i]));
            }

            // Sorting based on jointNames (first element of pair)
            std::sort(temp_pair.begin(), temp_pair.end(), 
                [](const std::pair<std::string, double>& a, const std::pair<std::string, double>& b){
                    return a.first < b.first;                        
                }
            );

            // Seprating back into 2 vectors
            for (size_t i = 0; i<temp_pair.size(); ++i){
                joint_names_[i] = temp_pair[i].first;
                joint_position_[i] = temp_pair[i].second;
            }

            

            /*
            RCLCPP_INFO(this->get_logger(), "~~~~~~~~~~~~~~Output~~~~~~~~~~~~~~~");
            RCLCPP_INFO(this->get_logger(), "  Joint_1: %f", output_joint_position_(0));
            RCLCPP_INFO(this->get_logger(), "  Joint_2: %f", output_joint_position_(1));
            RCLCPP_INFO(this->get_logger(), "  Joint_3: %f", output_joint_position_(2));
            RCLCPP_INFO(this->get_logger(), "  Joint_4: %f", output_joint_position_(3));
            */
            
            //std::cout<<"\n\n\nTyping: "<< output_joint_position_.data;

            //std::cout<<"\n Joint Names: "<<joint_names_[0]<<" "<<joint_names_[1]<<" "<<joint_names_[2]<<" "<<joint_names_[3];
            //std::cout<<"\n Joint Position: "<<joint_position_[0]<<" "<<joint_position_[1]<<" "<<joint_position_[2]<<" "<<joint_position_[3];
            
            // std::cout<<"\n Difference: "<<output_joint_position_.data[0] - joint_position_[0]<<" "<<output_joint_position_.data[1] - joint_position_[1]<<" "<<output_joint_position_.data[2] - joint_position_[2]<<" "<<output_joint_position_.data[3] - joint_position_[3];
            //std::cout<<"\n Difference" << diff_joint(joint_position_, output_joint_position_);

            //std::cout<<"\n\n\n Goal: " << goal_frame.p[0] << " " << goal_frame.p[1] << " " << goal_frame.p[2];
            //std::cout<<"\n Current: " << current_frame.p[0] << " " << current_frame.p[1] << " " << current_frame.p[2];
            //std::cout<<"\n Move Base To: " << goal_frame_cart.p[0]<< " "<< goal_frame_cart.p[1]<< " "<< goal_frame_cart.p[2];
            //std::cout<<"\n Same?: "<< diff_joint(goal_frame, current_frame);                        
            
            
            if (diff_joint(goal_frame, current_frame) > 0.05)
                {

                /////////MODIFY_CODE_HERE/////////
                
                // To get joint angle in radins provide goal location in cartisian space.
                output_joint_position_ = obj1_->get_desired_joint_angles(joint_position_, goal_frame);
                
                
                // If only arm needs to be moved. Means ball is within current reach of the arm.
                if (goal_frame_cart.p[0] == 10000.0 && goal_frame_cart.p[1] == 10000.0 && goal_frame_cart.p[2] == 10000.0)
                {
                    std::cout<<"\n Moving Arm Only\n";
                    // To move the arm through JointTrajectoryController
                    set_joint_trajectory(output_joint_position_);
                    current_frame.p = goal_frame.p;
                }

                // If both cart and arm needts to be moved.
                else{
                    std::cout<<"\n Moving Arm and Base\n";
                    // To move the cart through MecannumController
                    auto obj1 =RobotController();
                    obj1.publish_velocity(std::make_pair(goal_frame_cart.p[0], goal_frame_cart.p[1]), arm_move_time);
                    

                    // To move the arm through JointTrajectoryController
                    set_joint_trajectory(output_joint_position_);
                    current_frame.p = goal_frame.p;
                }                                            
                }
            
        }

                
        void set_joint_trajectory(KDL::JntArray set_joint_position_)
        {
            // std::cout<< "\n Moving Arm....";
            trajectory_msgs::msg::JointTrajectory traj;            

            // traj.header.stamp = this->now();
            //traj.header.frame_id  = "base_link";

            // traj.joint_names = joint_names_;
            traj.joint_names = {joint_names_[0], joint_names_[1], joint_names_[2], joint_names_[5]};
            
            trajectory_msgs::msg::JointTrajectoryPoint traj_point;
            
            // First move to the point of contact.
            // traj_point.positions = {set_joint_position_(0), set_joint_position_(1), set_joint_position_(2), set_joint_position_(3)};
            traj_point.positions = {set_joint_position_(0), set_joint_position_(1), set_joint_position_(2), set_joint_position_(3)};
            //traj_point.positions = {0.0, 1.0, 0.0, 0.0};
            //traj_point.time_from_start = rclcpp::Duration::from_seconds(2.0);
            traj_point.time_from_start = rclcpp::Duration::from_seconds(arm_move_time/2);
            traj.points.push_back(traj_point);
    
            // Next move arm back to generate force.
            //traj_point.positions = {-1.0, 1.0, 0.0, 0.0};           // NOTE: These are like the coordinates to move to, not the amount to move by.
            // traj_point.positions = {-1.0, set_joint_position_(1), set_joint_position_(2), set_joint_position_(3)};
            traj_point.positions = {set_joint_position_(0), set_joint_position_(1), set_joint_position_(2) - 1.0, set_joint_position_(3)};
            traj_point.time_from_start = rclcpp::Duration::from_seconds(arm_move_time/2 + arm_move_time/4);
            traj.points.push_back(traj_point);

            // Then move forward with speed.
            //traj_point.positions = {0.0, 1.0, 0.0, 0.0};
            // traj_point.positions = {0.0, set_joint_position_(1), set_joint_position_(2), set_joint_position_(3)};
            traj_point.positions = {set_joint_position_(0), set_joint_position_(1), set_joint_position_(2), set_joint_position_(3)};
            traj_point.time_from_start = rclcpp::Duration::from_seconds(arm_move_time/2 + arm_move_time/2);
            traj.points.push_back(traj_point);


            /*
            for (size_t i=0; i<joint_names_.size(); i++)
            {
                traj.joint_names.push_back(joint_names_[i]);

                trajectory_msgs::msg::JointTrajectoryPoint traj_point;
                //traj_point.positions.push_back(joint_position_[i]);
                traj_point.positions.push_back(set_joint_position_(i));

                traj.points.push_back(traj_point);
                
                //for (size_t i=0; i<joint_position_.size(); i++)
                //{
                //    traj_point.positions.push_back(joint_position_[i])
                //}
                
            }*/
            
            publish_state->publish(traj);

        }

        double diff_joint(KDL::Frame& goal, KDL::Frame& curr)
        {
            // double norm =  pow((out.data[0] - jn[0]), 2) + pow((out.data[1] - jn[1]), 2) + pow((out.data[2] - jn[2]), 2) + pow((out.data[3] - jn[3]), 2);
            double diff = pow((goal.p[0] - curr.p[0]), 2) + pow((goal.p[1] - curr.p[1]), 2) + pow((goal.p[2] - curr.p[2]), 2);  
            return std::sqrt(diff);
        }


        void process_data(const std_msgs::msg::String & msg) 
        {
            // ball_pose_count += 1;

            char delimiter = '@';
            // std::vector<std::string> result = split(msg.data, delimiter);
            result = split(msg.data, delimiter);

            //int time_to_impact, x_pred, y_pred, z_pred = msg.data.split("@");

            // std::cout<<"\n Got Data:\n"<<result.size()<<std::endl;
            // std::cout<<result[0]<<"\n"<<result[1]<<"\n"<<result[2]<<"\n"<<result[3]<<"\n";

            // Here we get error when converting the string to decimal, we observed it is due to the number being very small(close to 0). For this we coudl just make it 0.
            for (size_t i=0; i<result.size(); ++i){
                try{
                    std::stod(result[i]);       // It is intentionally being assigned to nothing.
                } catch (const std::out_of_range& e) {
                    result[i] = "0.0";
                }
            }

            // Time the ball will tage to reach the desired point.
            double time_to_impact = std::stod(result[0]);
            
            // The point arm needs to reach.
            double x_arm = std::stod(result[1]);
            double y_arm = std::stod(result[2]);
            double z_arm = std::stod(result[3]);

            // The point cart needs to reach.
            double x_cart = std::stod(result[4]);
            double y_cart = std::stod(result[5]);
            double z_cart = std::stod(result[6]);

            if (z_arm > 0.0) // and ball_pose_count > 4)
            {
                // std::cout<<"\n @@@@@@@@@@@@@@@@@@@@@@@@\n";
                // std::cout<<"\n @@@@@@@@@@@@@@@@@@@@@@@@\n";
                // std::cout<<"\n @@@@@@@@@@@@@@@@@@@@@@@@\n";
                // std::cout<<"\n @@@@@@@@@@@@@@@@@@@@@@@@\n";
                // std::cout<<"\n @@@@@@@@@@@@@@@@@@@@@@@@\n";
                // std::cout<<"\n @@@@@@@@@@@@@@@@@@@@@@@@\n";
                
                goal_frame.p = KDL::Vector(x_arm, y_arm, z_arm);   
                goal_frame_cart.p = KDL::Vector(x_cart, y_cart, z_cart);   
                arm_move_time = time_to_impact - 0.2;

                // ball_pose_count = -999999;
            }
        }

        
        std::vector<std::string> split(const std::string &s, char delimiter) {
            std::vector<std::string> tokens;
            std::string token;
            std::stringstream ss(s);

            while (std::getline(ss, token, delimiter)) {
                tokens.push_back(token);
            }

            return tokens;
        }
        
    private:    
        std::shared_ptr<IkSolver> obj1_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publish_state;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ball_coord;

        // To set where to reach in cartisian space.
        // Time to reach(time at which ball will reach desired point)
        double arm_move_time;

        // For Arm
        KDL::Frame goal_frame;

        // For Cart
        KDL::Frame goal_frame_cart;

        KDL::Frame current_frame;

        // Current Joint State
        std::vector<std::string> joint_names_;
        std::vector<double> joint_position_;

        // IK Derived Joint State
        KDL::JntArray output_joint_position_;
        
        //std::vector<std::string> name = std::vector<std::string>(5);
        // KDL::JntArray previous_ = KDL::JntArray(4);

        // To count the number of times position of ball is received
        //int ball_pose_count = 0;

        // To get trajectory data 
        std::vector<std::string> result;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<IkArm>()->init());
    rclcpp::shutdown();
    return 0;
}

























