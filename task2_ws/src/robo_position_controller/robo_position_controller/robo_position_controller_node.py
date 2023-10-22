#!/usr/bin/env python
#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg  import Odometry
from math import pow, atan2, sqrt
from rclpy.node import Node


def make_pose_position(x,y,z):
    pose = Pose().position
    pose.x = x
    pose.y = y
    pose.z = z
    return pose


PATHS = [
    [make_pose_position(x=1.0,y=0.0,z=0.0),make_pose_position(x=1.0,y=1.0,z=0.0),make_pose_position(x=0.0,y=1.0,z=0.0),make_pose_position(x=0.1,y=0.1,z=0.0)],

    [make_pose_position(x=1.0,y=1.0,z=0.0),make_pose_position(x=2.0,y=-1.0,z=0.0),make_pose_position(x=3.0,y=1.0,z=0.0),make_pose_position(x=4.0,y=-1.0,z=0.0),
     make_pose_position(x=5.0,y=1.0,z=0.0),make_pose_position(x=6.0,y=-1.0,z=0.0),make_pose_position(x=7.0,y=0.0,z=0.0),make_pose_position(x=0.1,y=0.1,z=0.0)]

    ]


class RoboPositionController(Node):

    def __init__(self):
        # Creates a node.
        super().__init__('robo_position_controller_node')
        self.moving_ = False
        self.at_goal_ = False
        self.path_to_follow_ = 0


        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.update_pose,10)
        self.orientation_subscriber = self.create_subscription(Imu, '/imu', self.update_angle,10)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.publisher_twist_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_error_ = self.create_publisher(Float32, '/pos_error', 10)


        self.pose = Pose().position
        self.goal_pose_ = Pose().position
        self.starting_point_ = Pose().position
        self.angle_to_goal_ = 0
        self.distance_tolerance_ = 0.1
        self.angle_tolerance_ = 0.1
        self.goal_angle_ = 0
        self.angular_velocity = Twist().angular
        self.yawn = 0
        self.goal_num_ = 0
        self.selected_mode = 0
        self.slope_ = 0
        self.intercept_ = 0

        timer_period = 0.09  # seconds
        self.timer_twist = self.create_timer(timer_period, self.move2goal)

    def update_pose(self, msg):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = msg.pose.pose.position
        #print(msg.pose.pose.x)
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        #print(self.pose)

    def update_angle(self, msg):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.angular_velocity = msg.angular_velocity
        #print(msg.orientation)
        self.yawn = self.quartet_to_yawn(msg.orientation)
        #print(f"{self.yawn} {self.angle_to_goal_}")
        self.angle_to_goal_ = self.steering_angle()-self.yawn

    def quartet_to_yawn(self, quartet):
        x= quartet.x
        y= quartet.y
        z= quartet.z
        w= quartet.w

        siny_cosp = 2*(w*z+x*y)
        cosy_cosp = 1-(2*(y*y+z*z))
        yawn = atan2(siny_cosp, cosy_cosp)
        return yawn

    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        #print(self.pose.x)
        #print(sqrt(pow((self.goal_pose_.x - self.pose.x), 2) + pow((self.goal_pose_.y - self.pose.y), 2)))
        return sqrt(pow((self.goal_pose_.x - self.pose.x), 2) +
                    pow((self.goal_pose_.y - self.pose.y), 2))

    def linear_vel(self, constant=0.2):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance()

    def steering_angle(self):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        #print(f"{atan2(self.goal_pose_.y - self.pose.y, self.goal_pose_.x - self.pose.x)}")
        return atan2(self.goal_pose_.y - self.pose.y, self.goal_pose_.x - self.pose.x)

    def angular_vel(self, constant=0.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle()-self.yawn)
    
    def distance_from_line(self, xp, yp):
    # function to calculate the distance of a point from a line 
    # defined by 2 points (self.starting_point_ and self.goal_point_)
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points

        return (abs((self.goal_pose_.x-self.starting_point_.x)*(self.starting_point_.y-yp)
                -(self.starting_point_.x-xp)*(self.goal_pose_.y-self.starting_point_.y))
                *sqrt(pow(self.goal_pose_.x-self.starting_point_.x,2)
                +pow(self.goal_pose_.y-self.starting_point_.y,2)))


    def move2goal(self):
        vel_msg = Twist()
        error_msg = Float32()
        if not self.moving_:
            
            self.at_goal_ = False
            self.selected_mode = int(input("What operation mode to use (1=xy, 2=xytheta, 3=path)"))
            self.get_logger().info(f'Selected mode of operation: {self.selected_mode}')

            if self.selected_mode == 1 or self.selected_mode == 2:
                # get user input for goal values, 2 mode has extra angle variables
                self.goal_pose_.x = float(input("Set your x goal: "))
                self.goal_pose_.y = float(input("Set your y goal: "))
                self.distance_tolerance_ = float(input("Set your distance tolerance: "))
                if self.selected_mode == 1:
                    self.get_logger().info(f'Goal: x:{self.goal_pose_.x} y:{self.goal_pose_.y}')

                if self.selected_mode == 2:
                    self.goal_angle_ = (3.14159/180)*float(input("Set your theta goal (deg): "))
                    self.angle_tolerance_ = float(input("Set your angle tolerance: "))
                    self.get_logger().info(f'Goal: x:{self.goal_pose_.x} y:{self.goal_pose_.y} theta:{self.goal_angle_ }')
                
                self.moving_ = True

            elif self.selected_mode == 3:
                # Get the input from the user.
                self.path_to_follow_ = int(input("Set your path to follow (1 = square, 2 = zigzag): "))-int(1)
                self.goal_num_= 0
                self.goal_pose_ = PATHS[self.path_to_follow_][self.goal_num_]
                self.get_logger().info(f'Selected path:{self.path_to_follow_+1}')

                self.moving_ = True

            if self.selected_mode in [1,2,3]:
                self.starting_point_.x = self.pose.x
                self.starting_point_.y = self.pose.y
                self.get_logger().info(f'starting point: x:{self.starting_point_.x} y:{self.starting_point_.y}')


        if self.moving_:
            # first we move to the goal
            if not self.at_goal_:
                # Porportional controller.
                # https://en.wikipedia.org/wiki/Proportional_control
                
                # first turn the robot to face the point
                # then start linear motion to the goal
                #print(f"{self.angle_to_goal_} {self.angular_velocity.z}")

                # check if we need angular movement so that we are heading towards the goal
                if abs(self.angle_to_goal_*180/3.14) >= 2:
                    vel_msg.angular.z = self.angular_vel()
                else: 
                    vel_msg.angular.z = 0.0
                # check if the angle to goal is small enough to allow forward movement
                if abs(self.angle_to_goal_*180/3.14) <= 45:
                    vel_msg.linear.x = self.linear_vel()
                else:
                    vel_msg.linear.x = 0.0

                # set rest of the Twist values to = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0

                #print(f"{abs(self.angle_to_goal_ - self.yawn)} {sqrt(pow((self.goal_pose_.x - self.pose.x), 2) + pow((self.goal_pose_.y - self.pose.y), 2))}")
                
                #check if we are close enough to the goal to stop linear motion
                if  self.euclidean_distance() <= self.distance_tolerance_:
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    self.get_logger().info(f'Goal reached: x:{self.pose.x} y:{self.pose.y}')

                    # if xy mode then this is the end of movement
                    if self.selected_mode == 1:
                        self.moving_ = False
                        self.at_goal_ = True
                        self.get_logger().info(f'XY move completed')
                    
                    # if thetha mode then this is the end of linear motion, start of
                    # angular motion to given theta
                    elif self.selected_mode == 2:
                        self.at_goal_ = True
                        self.get_logger().info(f'XY move completed, starting rotation to theta')

                    # if in path mode, check where in the path we are
                    elif self.selected_mode == 3:
                        self.goal_num_+= 1
                        self.get_logger().info(f'Goal number:{self.goal_num_}')

                        # if at the end of the path, end movement
                        if self.goal_num_ > len(PATHS[self.path_to_follow_]):
                            self.get_logger().info(f'Goals left:{len(PATHS[self.path_to_follow_]) -self.goal_num_}')
                            self.moving_ = False
                            self.at_goal_ = True

                        # if not at the end then pick next goal
                        else:
                            self.get_logger().info(f'Path completed')
                            self.goal_pose_ = PATHS[self.path_to_follow_][self.goal_num_]
                
                #calculate the distance from the line between starting point and goal
                error_msg.data = self.distance_from_line(self.pose.x, self.pose.y)
                self.publisher_error_.publish(error_msg)
            
            # this is only for theta mode. handles turning to the given theta at the goal
            else:
                vel_msg.linear.x = 0.0 # NO LINEAR MOTION ANYMORE
                # not close enough to the given theta
                if abs(self.yawn - self.goal_angle_) >= self.angle_tolerance_:
                    # Linear velocity in the x-axis.
                    #print(f"goal: {0.5* (self.goal_angle_-self.yawn)}")
                    vel_msg.angular.z = 0.1* (self.goal_angle_-self.yawn)
                # at the given theta
                else: 
                    vel_msg.angular.z = 0.0
                    self.moving_ = False
                    self.get_logger().info(f'Rotation completed: {self.yawn * 180/3.14159}')
                    #print(self.yawn * 180/3.14159)

            # Publishing our vel_msg
            self.publisher_twist_.publish(vel_msg)
                
        # Stopping our robot after the movement is over.
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.publisher_twist_.publish(vel_msg)
            self.moving_ = False
            self.at_goal_ = False


def main(args=None):
    rclpy.init(args=args)
    controller = RoboPositionController()
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
