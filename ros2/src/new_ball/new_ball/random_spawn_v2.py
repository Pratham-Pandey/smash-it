import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, ApplyLinkWrench, DeleteEntity, LinkRequest
from geometry_msgs.msg import Point, Quaternion, Wrench
import xacro
import time
import random

class BallSpawner(Node):

    def __init__(self):
        super().__init__('ball_spawner')
            
        # Initializing Clients    
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.apply_wrench_client = self.create_client(ApplyLinkWrench, '/apply_link_wrench')
        self.remove_wrench_client = self.create_client(LinkRequest, '/clear_link_wrenches')
        self.remove_client = self.create_client(DeleteEntity, '/delete_entity')
        
    
    def ball_loop(self):
        
        while True:
            # Function Calls
            self.spawn_ball()   # Spawn Ball
            #time.sleep(1)
            self.apply_force()  # Apply Force On Ball
            time.sleep(0.1)
            self.remove_force() # Remove Force From Ball
            time.sleep(4)
            self.delete_ball()  # Delete Ball
            time.sleep(4)
    
    def spawn_ball(self):
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            pass
            #self.get_logger().info('Spawn Ball Service not available, waiting again...')
        
        request = SpawnEntity.Request()
        
        request.name = "ball_1"
        
        # request.xml = open('/home/pratham/ros_development/smashit_ws/3_spawn_ball/ball_ws/src/ball_spawner/description/ball.urdf.xacro', 'r').read()
        request.xml = xacro.process_file('/home/pratham/ros_development/smashit_ws/3_spawn_ball/ball_ws/src/new_ball/description/ball.xacro').toxml()
        
        
        request.robot_namespace = "/"
        #request.initial_pose.position = Point(x= 0.5, y=8.0, z=3.0)
        #request.initial_pose.position = Point(x= 0.0, y=0.0, z=1.0)
        
        x_p = random.uniform(-1.0, 1.0)
        y_p = 9.0
        z_p = random.uniform(0.5, 1.5)
        
        
        request.initial_pose.position = Point(x= x_p, y=y_p, z=z_p)
        request.initial_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)        
        
        self.future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            pass
            #self.get_logger().info('Object spawned successfully')
        else:
            pass
            #self.get_logger().error('Failed to spawn object')
            
    def apply_force(self):
        while not self.apply_wrench_client.wait_for_service(timeout_sec=1.0):
            pass
            #self.get_logger().info('Apply Wrench Service not available, waiting again...') 
            
        request = ApplyLinkWrench.Request()
        request.link_name = "ball_1::base_link" # ball_1::base_link
        request.reference_frame = "" #"world"
        
        #request.wrench.force.x = 0.0  # Apply force in x-direction
        #request.wrench.force.y = -0.01
        #request.wrench.force.z = 0.0
        
        # Testing
        #request.wrench.force.x = 0.01  # Apply force in x-direction
        #request.wrench.force.y = 0.02       # -0.02
        #request.wrench.force.z = 0.05
        
        x_f = random.uniform(-10.0, 10.0)
        y_f = random.uniform(-55.0, -40.0)
        z_f = random.uniform(50.0, 70.0)
             
        request.wrench.force.x = x_f  # Apply force in x-direction
        request.wrench.force.y = y_f       # -0.02
        request.wrench.force.z = z_f
        
        request.duration.sec = -1  # Apply force for 1 second
        
        self.future = self.apply_wrench_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            pass
            #self.get_logger().info('Initial force applied successfully')
        else:
            pass
            #self.get_logger().error('Failed to apply initial force')
            
    def remove_force(self):
        while not self.remove_wrench_client.wait_for_service(timeout_sec=1.0):
            pass
            #self.get_logger().info('Remove Wrench Service not available, waiting again...') 
            
        request = LinkRequest.Request()
        request.link_name = "ball_1::base_link"
            
        self.future = self.remove_wrench_client.call_async(request)      # callign the service
        rclpy.spin_until_future_complete(self, self.future)             # blocks the execution of the current node until the request completes done
        
        if self.future.result() is not None:
            pass
            #self.get_logger().info('Force removed successfully')
        else:
            pass
            #self.get_logger().error('Failed to remove force')
        
    def delete_ball(self):
        while not self.remove_client.wait_for_service(timeout_sec=1.0):
            pass
            #self.get_logger().info('Delete Ball Service not available, waiting again...') 
            
        request = DeleteEntity.Request()
        request.name = "ball_1"    
        
        self.future = self.remove_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            pass
            #self.get_logger().info('Object deleted successfully')
        else:
            pass
            #self.get_logger().error('Failed to delete object')
        
        
            
            
def main(args=None):
    rclpy.init(args=args)
    spawner = BallSpawner()
    
    # Start ball loop
    spawner.ball_loop()
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
