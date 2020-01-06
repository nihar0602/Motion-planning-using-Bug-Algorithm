#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math     

SLOW = 0.3
FAST = 0.5

Thresold = .9
 

mode_ = 0

# INITIATING GLOBAL PARAMETERS 
slope = 0
c = 0
F = 3
FL = 3
FR = 3
yaw = 0
dist_ = 0


mode_list = {
        0: 'align to m-line',    
        1: 'GOAL SEEK MODE',
        2: 'WALL FOLLOW MODE',     
    }

#Calling laser-data

def callback_laser(msg):
    #converting 360 into 5 sectors = 360/5 = 72 each sector
    global sector_, F, FL, FR 
         
     
    sector_ = {
        'R':    min(min(msg.ranges[0:72]),3),
        'FR':   min(min(msg.ranges[73:144]),3),
        'F':    min(min(msg.ranges[145:216]),3),
        'FL':   min(min(msg.ranges[217:288]),3),
        'L':    min(min(msg.ranges[289:360]),3),
    }
    F = sector_['F']
    FL = sector_['FL']
    FR = sector_['FR']

# Selecting the mode: WallFollow or Goals
def switch_mode(mode):
    global mode_ 
    global mode_list

    mode_ = mode
    if mode is not mode_:
        print 'Wall follower - [%s] - %s' % (mode, mode_list[mode])
        mode_ = mode
    

# Some Codes for running the bot in some direction 

def straight():
    global F, FL, FR 
    vel_msg = Twist()

    if F > Thresold and FL > Thresold and FR > Thresold:
        print "Goal Seek Mode"
        vel_msg.linear.x = SLOW
        return vel_msg
    elif F > Thresold and FL < Thresold and FR > Thresold:
        print "Goal Seek Mode"
        vel_msg.linear.x = SLOW
        return vel_msg
    elif F > Thresold and FL < Thresold and FR < Thresold:
        print "Goal Seek Mode"
        vel_msg.linear.x = SLOW
        return vel_msg
    elif F < Thresold:
        print "Change to Wall Follower"
        switch_mode(2)
    
    vel_msg.linear.x = SLOW
    return vel_msg

def dist_to_mline(current, goal):
    num = math.fabs((goal.y - init.y) * current.x - (goal.x - init.x) * current.y + (goal.x * init.y) - (goal.y * init.x))
    den = math.sqrt(pow(goal.y - init.y, 2) + pow(goal.x - init.x, 2))
    back_ml = num / den
 
    return back_ml


       
                    


def Wall_follow():
    global F, FL, FR, current, goal
    vel_msg = Twist()

    print "hiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii", F, init.x <= current.x + .1, init.y <= current.y + .1
    if dist_to_mline(current, goal) < 0.15 and F > 1.5 and (init.x <= current.x + .1 or init.y <= current.y + .1):
        print "reached m:"
        switch_mode(0)
    elif F < Thresold and FL > Thresold and FR > Thresold:
        print "Front Obstacle"
        vel_msg.angular.z = 0.5
        return vel_msg
    elif F < Thresold and FL < Thresold and FR > Thresold:
        print "Front and Front Left Obstacle"
        vel_msg.angular.z = 0.4
        return vel_msg
    elif F < Thresold and FL > Thresold and FR < Thresold:
        print "Front and F.right Obstacle"
        vel_msg.angular.z = 0.4
        return vel_msg
    elif F < Thresold and FL < Thresold and FR < Thresold:
        print "Front and F.Left and F.right"
        vel_msg.angular.z = 0.4
        return vel_msg
    elif F > Thresold and FL < Thresold and FR < Thresold:
        print "Wall Follow 1"
        vel_msg.linear.x = SLOW
        return vel_msg
    elif F > Thresold and FL > Thresold and FR < Thresold:
        print "Wall Follow 2"
        vel_msg.linear.x = FAST
        return vel_msg
    elif F > Thresold and FL > Thresold and FR > Thresold:
        print "Wall Follow 23"
        vel_msg.linear.x = 0.2
        vel_msg.angular.z = -0.4
        return vel_msg

    else: print 'ERR!!', F, FL, FR
    return vel_msg     


def clb_odom(msg):
    global roll, pitch, yaw
    global current

    current = msg.pose.pose.position

    current_x = current.x 
    current_y = current.y 

    euler_orientation = msg.pose.pose.orientation
    quaternion = [euler_orientation.x, euler_orientation.y, euler_orientation.z, euler_orientation.w]
    (roll, pitch, yaw)= euler_from_quaternion(quaternion)
    #print 'Euler Y = [%s]' %yaw

def m_line(init, goal):
    slope = math.atan2(goal.y - init.y, goal.x - init.x)
    return slope

def distance_(current, goal):
    dist_ = math.sqrt(pow(goal.y- current.y, 2) + pow(goal.x - current.x, 2))
    return dist_



def align_bot():
    global yaw
    global c, current, goal, F
    global slope, init
    vel_msg = Twist()

    print 'check if mline', F, dist_to_mline(current, goal) < 0.15, init.x <= current.x + .1, init.y <= current.y + .1

    if F < Thresold:
        print "Switch to Wall Follow: 11111111111222222222222222233333333"
        switch_mode(2)
    elif dist_to_mline(current, goal) < 0.15 and (init.x <= current.x + .1 or init.y <= current.y + .1):
        print "Case_0 - On mline"
        if abs(yaw - slope) > 0.1:
            print "turning"
            vel_msg.linear.x = 0
            vel_msg.angular.z = .5
            return vel_msg
        else:   
            print "done turn"
            vel_msg.angular.z = 0
            switch_mode(1)
            return vel_msg

    print 'switch to WF from align'
    switch_mode(2)
    return vel_msg


# Main Function where publishers and subscribers are being called and initiated

def move():
    global yaw,  pub, slope, c, init, current, dist_, goal, back_ml, sector_ 
    

    vel_msg = Twist()

    rospy.init_node ('PA_1')
    pub = rospy.Publisher ('/cmd_vel', Twist, queue_size=[1])
    init = rospy.wait_for_message('/base_pose_ground_truth',Odometry).pose.pose.position
    current = init
    goal = rospy.wait_for_message('/homing_signal',PoseStamped).pose.position
    slope = m_line(init, goal)

    c = init.y - math.tan(slope) * init.x

    
    rospy.Subscriber ('/base_scan', LaserScan, callback_laser)

    rospy.Subscriber ('/base_pose_ground_truth', Odometry, clb_odom)

    
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        if distance_(goal, current) < 0.3:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            print "Done and Finish!!!!!"
        elif mode_ == 0:
            print "Aligning to M-line "
            vel_msg = align_bot()		
        elif mode_ == 1:
            print "GOAL SEEK MODE"
            vel_msg  = straight()
        elif mode_ == 2:
            print "WALL FOLLOW MODE"
            vel_msg = Wall_follow()
            pass
                    
        else:
            rospy.logerr('Error:404')
    
        pub.publish(vel_msg)
        print 'Distance to goal:' ,distance_(current, goal)
        print "Distance to m-line:'", dist_to_mline(current, goal)
         

        r.sleep()


if __name__ == "__main__":
    move()
    pass
   
    
    

