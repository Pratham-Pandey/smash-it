#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

class GetLocation : public rclcpp::Node
{
    public:
            
    private:
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<GetLocation>()->init());
    rclcpp::shutdown();
    return 0;
}
