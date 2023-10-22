#!/usr/bin/env python
#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg  import Odometry
from math import pow, atan2, sqrt
from rclpy.node import Node
import time



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


        self.pose = Pose().position
        self.goal_pose_ = Pose().position
        self.angle_to_goal_ = 0
        self.distance_tolerance_ = 0.1
        self.angle_tolerance_ = 0.1
        self.goal_angle_ = 0
        self.angular_velocity = Twist().angular
        self.yawn = 0
        self.goal_num_ = 0

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

    def angular_vel(self, constant=0.3):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle()-self.yawn)

    def move2goal(self):
        vel_msg = Twist()
        if not self.moving_:
            """Moves the turtle to the goal."""

            # Get the input from the user.
            self.path_to_follow_ = int(input("Set your path to follow (1 = square, 2 = zigzag): "))-int(1)
            self.goal_num_= 0
            self.goal_pose_ = PATHS[self.path_to_follow_][self.goal_num_]

            #print(self.euclidean_distance(goal_pose))
            self.moving_ = True

        if self.moving_:
            # first we move to the goal
            if not self.at_goal_:
                # Porportional controller.
                # https://en.wikipedia.org/wiki/Proportional_control
                
                # first turn the robot to face the point
                # then start linear motion to the goal
                #print(f"{self.angle_to_goal_} {self.angular_velocity.z}")
                if abs(self.angle_to_goal_*180/3.14) >= 2:
                    # Linear velocity in the x-axis.
                    vel_msg.angular.z = self.angular_vel()
                    vel_msg.linear.x = 0.0
                else: 
                    vel_msg.angular.z = 0.0

                if abs(self.angle_to_goal_*180/3.14) <= 45:
                    vel_msg.linear.x = self.linear_vel()
                else:
                    vel_msg.linear.x = 0.0

                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                # Publishing our vel_msg
                #print(f"{abs(self.angle_to_goal_ - self.yawn)} {sqrt(pow((self.goal_pose_.x - self.pose.x), 2) + pow((self.goal_pose_.y - self.pose.y), 2))}")
                if  self.euclidean_distance() <= self.distance_tolerance_:
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    self.goal_num_+= 1
                    if self.goal_num_ > len(PATHS[self.path_to_follow_]):
                        self.moving_ = False
                        self.at_goal_ = True
                    else:
                        self.goal_pose_ = PATHS[self.path_to_follow_][self.goal_num_]

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