import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

import numpy as np
import math

class GetPose(Node):
    def __init__(self):
        super().__init__('get_pose')
        
        # To keep track of number of times the trajectory of a ball has been published
        self.pub_count = 0
        
        # Ball coordinates
        self.x_pose = np.array([])
        self.y_pose = np.array([])
        self.z_pose = np.array([])
        
        # To get ball pose
        self.subscription = self.create_subscription(ModelStates, '/gazebo/model_states', self.process_data, 10)
        
        # To publish ball Trajectory
        self.publisher = self.create_publisher(String, 'ball_trajectory', 10)
        
        
        self.n_nearest_point = 5     # The number of points(representing future ball position) closest to the mobile robot to get.        
        self.min_z = 0.5             # The minimum amount of z value(height) the ball must be at to be considered for hitting.
        self.arm_length = 1.20 #1.35          # The length of the arm. # For now only arm length is considered. Might need to add cart height as well.
                
    def process_data(self, msg):
        
        if "ball_1" in msg.name:
            index = msg.name.index("ball_1")
            
            # self.get_logger().info("x: " + str(msg.pose[index].position.x) + "\ty: " + str(msg.pose[index].position.y) + "\tz: " + str(msg.pose[index].position.z))
            
            self.x_pose = np.append(self.x_pose, msg.pose[index].position.x)
            self.y_pose = np.append(self.y_pose, msg.pose[index].position.y)
            self.z_pose = np.append(self.z_pose, msg.pose[index].position.z)
            
            if (len(self.x_pose) > 2 and len(self.y_pose) > 2 and len(self.z_pose) > 2):
                if (self.z_pose[-2] > self.z_pose[-3] and self.z_pose[-2] > self.z_pose[-1] and self.pub_count == 0):
                                                                                          
                    x_vel = (self.x_pose[-2] - self.x_pose[-3])/0.05
                    y_vel = (self.y_pose[-2] - self.y_pose[-3])/0.05
                    z_vel = (self.z_pose[-2] - self.z_pose[-3])/0.05
                    
                    # Time to hit
                    #y_nearest = 0.0      # This is the point at which we need to hit the ball. Change it as per robot placement in the environment.
                    # To get position of the robot
                    bot_index = msg.name.index("nice_bot")        
                    bot_x = msg.pose[bot_index].position.x
                    bot_y = msg.pose[bot_index].position.y
                    bot_z = msg.pose[bot_index].position.z
                                        

                    point_time = 0.01  # This is the time gap between each generated points by the get_initial_trajectory(). It is 0.01 Seconds.
                    
                    x_data, y_data, z_data = self.get_initial_trajectory(0.72, 0.95, x_vel, y_vel, z_vel, self.x_pose[-2], self.y_pose[-2], self.z_pose[-2], 5)
                    # x_data, y_data, z_data = self.get_initial_trajectory(0.64, 0.95, x_vel, y_vel, z_vel, self.x_pose[-1], self.y_pose[-1], self.z_pose[-1])
                    
                    #print("Velocity(x, y, z), Position(x, y, z): ", x_vel, y_vel, z_vel, self.x_pose[-1], self.y_pose[-1], self.z_pose[-1])
                    # print("Predicted Trajectory:")
                    # print("X_data: ", x_data)
                    # print("Y_data: ", y_data)
                    # print("Z_data: ", z_data)
                    
                    nearest_index, distances = self.find_closest_element(x_data, y_data, z_data, np.array([bot_x, bot_y]))
                    
                    #print("Nearest Index: ", nearest_index[0], type(nearest_index[0])) 
                    #print("Output: ", nearest_index[0] * point_time, type(nearest_index[0] * point_time)) 
                                        
                    # t_hit = ([nearest_index[0] * point_time]) - 0.05    # Output in seconds. "-0.05" to account for the time elapsed after the second last point.                        
                    # t_hit = (nearest_index[0] * point_time) - 0.05                  # I think we also need to subtract time which has elapsed from begning to the point where we predict ball trajectory.
                    t_hit = ((nearest_index[0] - len(self.z_pose)) * point_time) - 0.05       # Here len(z_pose) will be used to count elapsed time.

                    arm_x_goal = x_data[nearest_index[0]]
                    arm_y_goal = y_data[nearest_index[0]]
                    arm_z_goal = z_data[nearest_index[0]]
                
                    #print("Bot Current XY: ", bot_x, bot_y)
                    
                    # if (distances[0] > self.arm_length):

                    if (distances[0] > (self.arm_length) or distances[0] < (self.arm_length - 0.2)):
                    # We will consider a sphere(3D) around the goal with a radius equal to the arm length. Now the edge of this sphere are the points through which the arm can reach the ball(goal point). But the mobile base cannot reach all the points on the edge of the sphere because it is fixed on ground(z=0). Therefore we will look for those points of the generated sphere which are having z=0.
                        
#                   (gx - x)^2 + (gy - y)^2 + (gz-0)^2 = r^2    # Equation of a 3D Sphere.
#                   (gx - x)^2 + (gy - y)^2 = r^2 - (gz-0)^2    # Solving it converts it to equation of a 2D sphere. 
#                   Radius_2D_Circle = root(r^2 - (gz-0)^2)     # Radius of the circle formed when the 3D sphere intersects the plane(z=0).                      
#                   Next we could generate the coordinate at 2 points of this circle. first at 0 degree and second at 180 degree(only these 2 points because be want to hit the ball from exactly left or right side).

                        # theta = [0, 180]
                        theta = [0, np.pi]
                        bot_goal = []
                        
                        radius_2d_circle = math.sqrt(self.arm_length**2 - (0 - z_data[nearest_index[0]])**2)
       
                        for i in theta:                    
                            temp_x_goal = x_data[nearest_index[0]] + radius_2d_circle * np.cos(i)                       
                            temp_y_goal = y_data[nearest_index[0]] + radius_2d_circle * np.sin(i)
                            temp_z_goal = 0      # Means the mobile robot is on ground.
                            bot_goal.append((temp_x_goal, temp_y_goal, temp_z_goal))   # After adding data at both the points(0, 180 degree),we will calculate distance of actual mobile base position to these points and will select the closest point.
                        
                        dist_p1_0 = np.sqrt((bot_x - bot_goal[0][0])**2 + (bot_y - bot_goal[0][1])**2) 
                        dist_p2_180 = np.sqrt((bot_x - bot_goal[1][0])**2 + (bot_y - bot_goal[1][1])**2)                        

                        if (dist_p1_0 < dist_p2_180):
                            bot_x_goal = bot_goal[0][0]
                            bot_y_goal = bot_goal[0][1]
                            bot_z_goal = bot_goal[0][2]       
                        else:
                            bot_x_goal = bot_goal[1][0]
                            bot_y_goal = bot_goal[1][1]
                            bot_z_goal = bot_goal[1][2]       

                    else:           # Means some points are within reach of arm from the current mobile robot position.
                                                
                        # Setting mobile base position to "10000" means no change as the ball is already within arm's reach.                                                   
                        bot_x_goal = "10000.0"
                        bot_y_goal = "10000.0"
                        bot_z_goal = "10000.0"                                                
                    
                        
                    # For viewing
                    print("Time to reach: ", t_hit)
                    print("Mobile base to reach: ", bot_x_goal, bot_y_goal, bot_z_goal)
                    print("Arm to reach: ", arm_x_goal, arm_y_goal, arm_z_goal)

                    print("~~~~~~~~~~~~")
                    print("\n\n")
                
                            
                    # Publish Data
                    msg = String()
                    msg.data = str(t_hit) + "@" + str(arm_x_goal)+ "@" + str(arm_y_goal)+ "@" + str(arm_z_goal) + "@" + str(bot_x_goal)+ "@" + str(bot_y_goal)+ "@" + str(bot_z_goal)
                    
                    self.publisher.publish(msg)
                    self.pub_count = self.pub_count + 1
                    
                        
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    
                    #t_hit = (nearest_index * point_time) - 0.05      # Output in seconds. "-0.05" to account for the time elapsed after the second last point.
                    #x_goal = x_data[nearest_index]        
                    #y_goal = y_data[nearest_index]
                    #z_goal = z_data[nearest_index]                                                                        
        
        else:
            # print("\n\n\nActual Coordinates: ")
            # print("X:", self.x_pose.tolist())
            # print("Y:", self.y_pose.tolist())
            # print("Z:", self.z_pose.tolist())
            
            self.x_pose = np.array([])
            self.y_pose = np.array([])  
            self.z_pose = np.array([])                
            
            self.pub_count = 0
    
    def find_closest_element(self, x, y, z, point):
        
        # Removing those points having z value(height) below a certain threshold. 
        # Instead of removing them, we are setting them very high value so that when calculating distance to these points,they would be far away. 
        
        x = np.array(x)
        y = np.array(y)        
        z = np.array(z)
        
        idx = np.where(z < self.min_z)[0]
        x[idx] = 10000
        y[idx] = 10000
        z[idx] = 10000
        
        # And Those above arm length(as arm wont be able to reach it even if it is directly under it).
        idx = np.where(z > self.arm_length)[0]
        x[idx] = 10000
        y[idx] = 10000
        z[idx] = 10000
        
                
        # Calculating Euclidean Distance between the robot and the expected ball position.
        distances = np.sqrt((x - point[0])**2 + (y - point[1])**2)        
        
        nearest_indices = np.argsort(distances)[:self.n_nearest_point]
        distances = np.sort(distances)[:self.n_nearest_point]
                      
        return nearest_indices, distances
      
                
    def get_initial_trajectory(self, e_1=0.8, q_1=1.0, vx0_inp=1.0, vy0_inp=1.95, vz0_inp=4.4, x_start=0, y_start=0, z_start=1, sim_time=5):     # Continue from here
        # Constants
        g = 9.81  # gravity (m/s^2)
        e = e_1 #0.8  # coefficient of restitution
        q = q_1     # Horizontal velocity coefficient

        x0, y0, z0 = x_start, y_start, z_start  # initial position (m)

        vx0 = vx0_inp
        vy0 = vy0_inp
        vz0 = vz0_inp

        t_step = 0.01  # time step for simulation (s)
        t_max = sim_time  # Default: 5  # max time for simulation (s)

        # Initialize lists to store trajectory points
        x_vals = [x0]
        y_vals = [y0]
        z_vals = [z0]

        # Simulation loop
        t = 0
        x, y, z = x0, y0, z0
        vx, vy, vz = vx0, vy0, vz0

        while t < t_max:
            t += t_step
            x += vx * t_step
            y += vy * t_step
            z += vz * t_step - 0.5 * g * t_step ** 2
            vz -= g * t_step

            if z <= 0:  # Ball hits the ground
                z = 0
                vz = -e * vz

                vx = q * vx
                vy = q * vy
                #vx = vx * q    # Added

            x_vals.append(x)
            y_vals.append(y)
            z_vals.append(z[0] if isinstance(z, np.ndarray) else z )

            #if len(y_vals) > 1 and y_vals[-1] == y_vals[-2] and z_vals[-1] == z_vals[-2]:
            if len(y_vals) > 1 and y_vals[-1] == y_vals[-2] and z_vals[-1] == z_vals[-2] and x_vals[-1] == x_vals[-2]:
                break  # Stop simulation if ball stops bouncing

        return x_vals, y_vals, z_vals
                
                
def main(args=None):
    rclpy.init(args=args)
    obj_1 = GetPose()
    
    rclpy.spin(obj_1)
    obj_1.destory_node()
    rclpy.shutdown()
    
    
if __name__== '__main__':
    main()
    
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
