import math
import rospy as ros
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


__author__ = "Gabriel Urbain"
__copyright__ = "Copyright 2018, IDLab, UGent"

__license__ = "MIT"
__version__ = "1.0"
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be"
__status__ = "Education"
__date__ = "October 15th, 2018"


class SquareMove(object):
    """
    This class is an abstract class to control a square trajectory on the turtleBot.
    It mainly declare and subscribe to ROS topics in an elegant way.
    """

    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "square_move"
        self.odom_sub_name = "/odom"
        self.vel_pub_name = "/cmd_vel"
        self.vel_pub = None
        self.odometry_sub = None

        # ROS params
        self.pub_rate = 0.01
        self.queue_size = 2

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None

    def start_ros(self):

        # Create a ROS node with a name for our program
        ros.init_node(self.node_name, log_level=ros.INFO)

        # Define a callback to stop the robot when we interrupt the program (CTRL-C)
        ros.on_shutdown(self.stop_robot)

        # Create the Subscribers and Publishers
        self.odometry_sub = ros.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

    def stop_robot(self):

        # Get the initial time
        self.t_init = time.time()

        # We publish for a second to be sure the robot receive the message
        while time.time() - self.t_init < 1 and not ros.is_shutdown():

            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)

        sys.exit("The process has been interrupted by the user!")

    def move(self):
        """ To be surcharged in the inheriting class"""

        while not ros.is_shutdown():
            time.sleep(1)

    def __odom_ros_sub(self, msg):

        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):

        self.vel_pub.publish(msg)




class SquareMoveVel(SquareMove):
    """
    This class implements a open-loop square trajectory based on velocity control. HOWTO:
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
     - Start this node on your computer:
            $ python eced3901_dt1.py vel
    """

    def __init__(self):

        super(SquareMoveVel, self).__init__()

    def go_forward(self, duration, speed):

        # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not ros.is_shutdown():

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

    def turn(self, duration, ang_speed):

         # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait 2 sec (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not ros.is_shutdown():

            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = ang_speed
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

    def move(self):

        self.go_forward(2, 5)
        self.turn(3.5, 0.5)
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)
        self.go_forward(2, 0.5)
        self.stop_robot()


    def move_of(self, d, speed=0.1143):

        x_init = self.odom_pose.position.x
        y_init = self.odom_pose.position.y
	z_init = self.get_z_rotation(self.odom_pose.orientation)

        # Set the velocity forward until distance is reached
        while math.sqrt((self.odom_pose.position.x - x_init)**2 + \
             (self.odom_pose.position.y - y_init)**2) < d and not ros.is_shutdown():

            sys.stdout.write("\r [MOVE] The robot has moved of {:.2f}".format(math.sqrt((self.odom_pose.position.x - x_init)**2 + \
            (self.odom_pose.position.y - y_init)**2)) +  "m over " + str(d) + "m")
            sys.stdout.flush()

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            
	    #print z_init
	    
	    #Value used in the angle judgement function
	    angle_dif = abs(self.get_z_rotation(self.odom_pose.orientation)) - abs(z_init)
	    y_dif = abs(self.odom_pose.position.y)-abs(y_init)
	    x_dif = abs(self.odom_pose.position.x)-abs(x_init)
	    angle = self.get_z_rotation(self.odom_pose.orientation)

	    #Print the difference value between current and init values
	    #And the angle of robot move
	    print ("\n")
	    sys.stdout.write("Y difference: {:f}".format(y_dif) + "\t" )
	    sys.stdout.write("X difference: {:f}".format(x_dif) + "\t" )
	    sys.stdout.write("Angle difference: {:f}".format(angle_dif) + "\n" )
	    sys.stdout.write("Angle : {:f}".format(self.get_z_rotation(self.odom_pose.orientation)) + "\n" )
            sys.stdout.flush()

	    #Set up the correction speed for robot judge the angle
	    correct_speed = 0.018

	    #Print the difference value between current and init
	    """
	    This part to help the robot to re-adjust the angle that robot forward,
	    help the robot go back to the correct line.
	    Through the difference in each axial to destinate the postion of robot,
	    then choose the right movement to turn the robot by the fixed angle
	    """
	    #Set the maximum of correction amount
	    if abs(angle_dif) >= 0.018 :
		# When robot need to fix turnning right
	        if  angle_dif > 0 :
		    # When robot move from far point to origin in y-axiiis 
		    # When the angle is positive, let robot turn in negative speed
		    if x_dif < 0 and y_dif < 0 and angle > 0:
		      self.turn_of(abs(angle_dif),-correct_speed)
		    # When the angle is negative, let robot turn in positive speed
		    elif x_dif < 0 and y_dif < 0 and angle < 0:
		      self.turn_of(abs(angle_dif),correct_speed)

		    # When robot move from origin to far point in y-axis
	            # and move from far point to origin x-axis
		    # When the angle is positive, let robot turn in negative speed
		    elif x_dif < 0 and y_dif > 0 and angle > 0:
		      self.turn_of(abs(angle_dif),-correct_speed)
		    # When the angle is negative, let robot turn in positive speed
		    elif x_dif < 0 and y_dif > 0 and angle < 0:
		      self.turn_of(abs(angle_dif),correct_speed)

		    # When robot move from far point to origin in y-axis
	            # and move from origin to far point x-axis
		    # When the angle is positive, let robot turn in negative speed
		    elif x_dif > 0 and y_dif < 0 and angle > 0:
		      self.turn_of(abs(angle_dif),-correct_speed)
		    # When the angle is negative, let robot turn in positive speed
		    elif x_dif > 0 and y_dif < 0 and angle < 0:
		      self.turn_of(abs(angle_dif),correct_speed)

		    # When robot move from origin to far point in y-axis
	            # and move from origin to far point in x-axis
		    elif x_dif > 0 and y_dif > 0 and angle < 0:
		    # When the angle is negative, let robot turn in positive speed
		      self.turn_of(abs(angle_dif),correct_speed)
		    elif x_dif > 0 and y_dif > 0 and angle > 0:
		    # When the angle is positive, let robot turn in negative speed
		      self.turn_of(abs(angle_dif),-correct_speed)
		    else:
		      continue

		# When robot need to fix turnning right
		elif angle_dif < 0 :
		    # When robot move from far point to origin in y-axiiis 
		    if x_dif < 0 and y_dif < 0 and angle > 0 :
		    # When the angle is positive, let robot turn in positive speed
		      self.turn_of(abs(angle_dif),correct_speed)
		    elif x_dif < 0 and y_dif < 0 and angle < 0 :
		    # When the angle is negative, let robot turn in negative speed
		      self.turn_of(abs(angle_dif),-correct_speed)

		    # When robot move from far point to origin in y-axis
	            # and move from origin to far point x-axis
		    elif x_dif < 0 and y_dif > 0 and angle < 0 :
		    # When the angle is negative, let robot turn in negative speed
		      self.turn_of(abs(angle_dif),-correct_speed)
		    elif x_dif < 0 and y_dif > 0 and angle > 0 :
		    # When the angle is positive, let robot turn in positive speed
		      self.turn_of(abs(angle_dif),correct_speed)

		    # When robot move from far point to origin in y-axis
	            # and move from origin to far point x-axis
		    elif x_dif > 0 and y_dif < 0 and angle > 0 :
		    # When the angle is positive, let robot turn in positive speed
		      self.turn_of(abs(angle_dif),correct_speed)
		    elif x_dif > 0 and y_dif < 0 and angle < 0 :
		    # When the angle is negative, let robot turn in negative speed
		      self.turn_of(abs(angle_dif),-correct_speed)

		    # When robot move from origin to far point in y-axis
	            # and move from origin to far point in x-axis
		    elif x_dif > 0 and y_dif > 0 and angle < 0 :
		    # When the angle is negative, let robot turn in negative speed
		      self.turn_of(abs(angle_dif),-correct_speed)
		    elif x_dif > 0 and y_dif > 0 and angle > 0 :
		    # When the angle is positive, let robot turn in positive speed
		      self.turn_of(abs(angle_dif),correct_speed)
		    else:
		      continue
		else:
		    continue
	    else:
		continue
	    time.sleep(self.pub_rate)
        sys.stdout.write("\n")
    def turn_of(self, a, ang_speed):

        # Convert the orientation quaternion message to Euler angles
        a_init = self.get_z_rotation(self.odom_pose.orientation)
        print a_init
        # Set the angular velocity forward until angle is reached
        while (abs(self.correct_angle(self.get_z_rotation(self.odom_pose.orientation) - a_init))) <= a and not ros.is_shutdown():

            sys.stdout.write("\r [TURN] The robot has turned of {:.5f}".format(abs(self.get_z_rotation(self.odom_pose.orientation) - \
                 a_init)) + "rad over {:.5f}".format(a) + "rad")
            sys.stdout.flush()
            #print (self.get_z_rotation(self.odom_pose.orientation) - a_init)
		
            msg = Twist()
            msg.angular.z = ang_speed
            msg.linear.x = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    def move(self):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not ros.is_shutdown():
            time.sleep(0.1)

	ang_speed = 0.1451

        # Implement main instructions
        self.move_of(0.9)
        self.turn_of(math.pi/2,ang_speed)
        self.move_of(0.45)
        self.turn_of(math.pi/2,ang_speed)
        self.move_of(0.9)
        self.turn_of(math.pi/2,ang_speed)
        self.move_of(0.45)

	self.turn_of(math.pi/2,ang_speed)
	self.turn_of(math.pi/2,ang_speed)
	self.move_of(0.45)
        self.turn_of(math.pi/2,-ang_speed)
        self.move_of(0.9)
        self.turn_of(1.5411,-ang_speed)
        self.move_of(0.45)
        self.turn_of(math.pi/2,-ang_speed)
        self.move_of(0.9)
        self.turn_of(math.pi/2,-ang_speed)
        self.turn_of(math.pi/2,-ang_speed)
        self.stop_robot()




if __name__ == '__main__':

    # Choose the example you need to run in the command line
    if len(sys.argv) > 1:

        if sys.argv[1] == "vel":
            r = SquareMoveVel()

        elif sys.argv[1] == "odom":
            r = SquareMoveOdom()

        else:
            sys.exit(-1)

    else:
        sys.exit(-1)

    # Listen and Publish to ROS + execute moving instruction
    r.start_ros()
    r.move()





