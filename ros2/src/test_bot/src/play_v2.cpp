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

// Temp
#include "gazebo_msgs/srv/set_entity_state.hpp"

// For getting the robot position(For coordinate transform)
/*
class GetPose : public rclcpp::Node
{
public:
    GetPose() : Node("get_pose")
    {
        subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 10, std::bind(&GetPose::update, this, std::placeholders::_1));
    }

    std::pair<double, double> coord;  // For x and y coordinate
    KDL::Frame arm_coord;

private:
    void update(const gazebo_msgs::msg::ModelStates & msg)
        {
            std::pair<bool, double> p1 = find_val(msg.name, "nice_bot");

            if (p1.first == true)
            {
                double x_pose = msg.pose[p1.second].position.x;
                double y_pose = msg.pose[p1.second].position.y;
                double z_pose = msg.pose[p1.second].position.z;

                double x_roll = msg.pose[p1.second].orientation.x;
                double y_roll = msg.pose[p1.second].orientation.y;
                double z_roll = msg.pose[p1.second].orientation.z;
                double w_roll = msg.pose[p1.second].orientation.w;
                
                arm_coord.p = KDL::Vector(x_pose, y_pose, z_pose);
                arm_coord.M = KDL::Rotation::Quaternion(x_roll, y_roll, z_roll, w_roll);

                // rotation = KDL::Rotation::Quaternion(x, y, z, w);
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
*/

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

// To get joint state of the robot.
class GetJointStates : public rclcpp::Node{
    public:
        GetJointStates() : Node("get_joint_states")
        {
            joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&GetJointStates::process_joint_angles, this, std::placeholders::_1)); 
        }

        // For storing Joint Names and Joint Position
        std::vector<std::string> joint_names_;
        std::vector<double> joint_position_;

    private:

        void process_joint_angles(const sensor_msgs::msg::JointState & msg)
        {
            joint_names_ = msg.name;
            joint_position_ = msg.position;

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



        }        

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_;    // Subscriber to get current joint state(for arms).  
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

class GetLocation : public rclcpp::Node
{
    public:
        GetLocation() : Node("get_location")
        {
            std::cout<<"\nStarting: ";
        }

        std::shared_ptr<rclcpp::Node> init()
        {
            // obj_2 = std::make_shared<IkSolver>();
            obj_2 = std::make_shared<IkSolver>(shared_from_this());            

            location = this->create_subscription<std_msgs::msg::String>("ball_trajectory", 10, std::bind(&GetLocation::process_data, this, std::placeholders::_1));            
            
            obj_1 = std::make_shared<GetJointStates>();    // For updating Joint States.  
            
            
            current_frame.p = KDL::Vector(0.0, 0.0, 0.0);   // IMPORTANT: Change it to get latest robot arm position

            // To set the desired joint angles for Joint_Trajectory_Controller
            publish_state = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_position_controller/joint_trajectory", 10);

            // To change the position of robot.
            spawn_client = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");

            return shared_from_this();
        }

    private:
        void process_data(const std_msgs::msg::String & msg)
        {
            std::cout<<"Data: "<<msg.data<<std::endl;

            char delimiter = '@';
            std::vector<std::string> split_data = split(msg.data, delimiter);

            // Here we get error when converting the string to decimal, we observed it is due to the number being very small(close to 0). For this we could just make it 0.
            for (size_t i=0; i<split_data.size(); ++i){
                try{
                    std::stod(split_data[i]);       // It is intentionally being assigned to nothing.
                } catch (const std::out_of_range& e) {
                    split_data[i] = "0.0";
                }
            }

            // The point arm needs to reach.
            double x_arm = std::stod(split_data[1]);
            double y_arm = std::stod(split_data[2]);
            double z_arm = std::stod(split_data[3]);

            // The point cart needs to reach.
            double x_cart = std::stod(split_data[4]);
            double y_cart = std::stod(split_data[5]);
            double z_cart = std::stod(split_data[6]);

            if (z_arm > 0.0)
            {   
                goal_frame.p = KDL::Vector(x_arm, y_arm, z_arm);   
                goal_frame_cart.p = KDL::Vector(x_cart, y_cart, z_cart);   

                goal_frame = goal_frame_cart.Inverse() * goal_frame;

                // arm_move_time = std::stod(split_data[0]) - 0.2;  // Here 0.2 is a parameter to slow or fast the movement of the arm.
                arm_move_time = std::stod(split_data[0]);

                // Now call the "get_current_joint_angles()" to move the robot.

                rclcpp::spin_some(obj_1);   // Update Joint States

                // Current Joint Names and Joint Position
                std::vector<std::string> joint_names = obj_1->joint_names_;
                std::vector<double> joint_position = obj_1->joint_position_;

                // Move Bot
                move_bot(joint_names, joint_position);
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

        void move_bot(const std::vector<std::string> & j_names, const std::vector<double> j_position)
        {
            if (diff_joint(goal_frame, current_frame) > 0.05)
                {

                /////////MODIFY_CODE_HERE/////////
                
                // If only arm needs to be moved. Means ball is within current reach of the arm.
                if (goal_frame_cart.p[0] == 10000.0 && goal_frame_cart.p[1] == 10000.0 && goal_frame_cart.p[2] == 10000.0)
                {
                    std::cout<<"\n Moving Arm Only\n";

                    // To get joint angle in radians provide goal location in cartisian space.
                    output_joint_position_ = obj_2->get_desired_joint_angles(j_position, goal_frame);

                    // To move the arm through JointTrajectoryController
                    set_joint_trajectory(output_joint_position_, j_names);
                    current_frame.p = goal_frame.p;
                }

                // If both cart and arm needs to be moved.
                else{
                    std::cout<<"\n Moving Arm and Base\n";
                    
                    // Temp: To spawn the bot directly near point of strike.

                    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

                    request->state.name = "nice_bot";
    
                    request->state.pose.position.x = goal_frame_cart.p[0];
                    request->state.pose.position.y = goal_frame_cart.p[1];
                    request->state.pose.position.z = 0.0;

                    request->state.reference_frame = "world";

                    auto spawn_result = spawn_client->async_send_request(request);


                    // ~~~~~~~~~~~~~~~~~~~
                    
                    // To move the cart through MecannumController
                    // auto obj3 =RobotController();
                    // obj3.publish_velocity(std::make_pair(goal_frame_cart.p[0], goal_frame_cart.p[1]), arm_move_time);
                    
                    // To get joint angle in radians provide goal location in cartisian space.
                    output_joint_position_ = obj_2->get_desired_joint_angles(j_position, goal_frame);

                    // To move the arm through JointTrajectoryController
                    set_joint_trajectory(output_joint_position_, j_names);
                    current_frame.p = goal_frame.p;
                }                                            
            }   
        }

        double diff_joint(KDL::Frame& goal, KDL::Frame& curr)
        {
            // double norm =  pow((out.data[0] - jn[0]), 2) + pow((out.data[1] - jn[1]), 2) + pow((out.data[2] - jn[2]), 2) + pow((out.data[3] - jn[3]), 2);
            double diff = pow((goal.p[0] - curr.p[0]), 2) + pow((goal.p[1] - curr.p[1]), 2) + pow((goal.p[2] - curr.p[2]), 2);  
            return std::sqrt(diff);
        }

        void set_joint_trajectory(KDL::JntArray set_joint_position_, const std::vector<std::string> & j_names)
        {
            // std::cout<< "\n Moving Arm....";
            trajectory_msgs::msg::JointTrajectory traj;            

            traj.joint_names = {j_names[0], j_names[1], j_names[2], j_names[5]};
            
            trajectory_msgs::msg::JointTrajectoryPoint traj_point;
            
            // First move to the point of contact.
            //traj_point.positions = {set_joint_position_(0), set_joint_position_(1), set_joint_position_(2), set_joint_position_(3)};
            traj_point.positions = {set_joint_position_(0), set_joint_position_(1), 0, set_joint_position_(3)};
            // traj_point.time_from_start = rclcpp::Duration::from_seconds(arm_move_time/2);
            traj_point.time_from_start = rclcpp::Duration::from_seconds(arm_move_time * 0.7);
            traj.points.push_back(traj_point);
    
            // Next move arm back to generate force.
            // NOTE: These are like the coordinates to move to, not the amount to move by.
            //traj_point.positions = {set_joint_position_(0), set_joint_position_(1), 0 , set_joint_position_(3) + 1.0};
            traj_point.positions = {set_joint_position_(0) + 1.0, set_joint_position_(1) + 1.0, 0 , set_joint_position_(3)};
            // traj_point.time_from_start = rclcpp::Duration::from_seconds(arm_move_time/2 + arm_move_time/4);
            traj_point.time_from_start = rclcpp::Duration::from_seconds(arm_move_time * 0.95);
            traj.points.push_back(traj_point);

            // Then move forward with speed.
            traj_point.positions = {set_joint_position_(0), set_joint_position_(1), 0, set_joint_position_(3)};
            // traj_point.time_from_start = rclcpp::Duration::from_seconds(arm_move_time/2 + arm_move_time/2);
            traj_point.time_from_start = rclcpp::Duration::from_seconds(arm_move_time * 1.1);
            traj.points.push_back(traj_point);
            
            // Publish Data 
            publish_state->publish(traj);

        }


        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr location;    // Subscriber for retriving the estimated position to move the arm and the mobile base to. Also the estimated time at which the ball will reach the arm position.

        double arm_move_time;           // Time the ball will take to reach the desired point.
        
        KDL::Frame goal_frame;          // Goal for Arm
        KDL::Frame current_frame;       // Curent position of Arm

        KDL::Frame goal_frame_cart;     // Goal for Cart        

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publish_state;  // Publisher to publish data for Arm Controller.
        rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr spawn_client;   // Client to request change in robot position.

        std::shared_ptr<GetJointStates> obj_1;
        std::shared_ptr<IkSolver> obj_2;

        // IK Derived Joint State
        KDL::JntArray output_joint_position_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<GetLocation>()->init());
    rclcpp::shutdown();
    return 0;
}
