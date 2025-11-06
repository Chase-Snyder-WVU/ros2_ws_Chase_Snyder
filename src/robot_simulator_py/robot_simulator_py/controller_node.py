# @file controller_node.py
#
# @author Chase Snyder
#
# @brief simple state machine py controller
#        for publishing velocity commands 
#        from odometry node to drive a 2mx2m square.
#

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        #Publish velocity commands to /cmd_vel ###############
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Square corners in odom frame
        #0,1,2,3 = which edge of the square
        #fixed speed (m/s)
        self.tolerance = 0.05
        self.state = 0          
        self.speed = 0.2        

        #Run control loop
        self.timer = self.create_timer(1.0, self.on_timer)

        #Display when controller starts
        self.get_logger().info("controller_node started (simple square path)")
        
        #start odometry node
        cmd =Twist()
        cmd.linear.x = self.speed
        self.cmd_pub.publish(cmd)

    def on_timer(self):

        #Get odom to base_link transform (current pose)
        try:
            t = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(f'Could not transform odom to base_link: {ex}')
            return

        #Get transfom data
        x = t.transform.translation.x
        y = t.transform.translation.y

        cmd = Twist()

        ### State machine ###

        if self.state == 0:
            #Edge 0, (0,0) to (2,0), move in the +x
            #Stop when reaching the goal with tolerance.
            if x >= 2.0 - self.tolerance:
                #Change state and display state change
                self.get_logger().info("Reached (2,0), switching to state 1")
                self.state = 1
            else:
                #otherwise command movement toward goal
                cmd.linear.x = self.speed   

        elif self.state == 1:
            #Edge 1, (2,0) to (2,2), move in the +y
            #Stop when reaching the goal with tolerance.
            if y >= 2.0 - self.tolerance:
                #Change state and display state change
                self.get_logger().info("Reached (2,2), switching to state 2")
                self.state = 2
            else:
                #otherwise command movement toward goal
                cmd.linear.y = self.speed   

        elif self.state == 2:
            #Edge 2, (2,2) to (0,2), move in the -x
            #Stop when reaching the goal with tolerance.
            if x <= 0.0 + self.tolerance:
                #Change state and display state change
                self.get_logger().info("Reached (0,2), switching to state 3")
                self.state = 3
            else:
                #otherwise command movement toward goal
                cmd.linear.x = -self.speed

        elif self.state == 3:
            #Edge 3, (0,2) to (0,0), move in the -y
            #Stop when reaching the goal with tolerance.
            if y <= 0.0 + self.tolerance:
                #Change state and display state change
                self.get_logger().info("Reached (0,0), switching to state 0")
                self.state = 0
            else:
                 #otherwise command movement toward goal
                cmd.linear.y = -self.speed  

        #Send the command.
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    #Stop program
    rclpy.shutdown()


if __name__ == '__main__':

    main()
