#!/usr/bin/env python
import roslib
import rospy
import math
import random
import time
import numpy
import tf
from tf.transformations import euler_from_quaternion
import message_filters 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from heapq import *
from std_msgs.msg import String

currentpos_x= -8
currentpos_y =-2
flag = 0
ctr = 0
goal_x = 4.5
goal_y = 9.0
startx = -8.0
starty = -2.0
flag_r = 0
flag_t = 0
path_goal2start = []
path_start2goal = []
temp_pos = []
Finalloc_x =0
Finalloc_y = 0
PathMap = numpy.array([
    [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
    [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
    [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
    [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
    [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
    [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
    [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
    [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
    [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]])


def Distance_cost(x, y):
    return (y[0] - x[0]) ** 2 + (y[1] - x[1]) ** 2

def Astar_Algo(array, s, g):
    print s
    print g
    s_x = s[0]
    s_y = s[1]
    g_x = g[0]
    g_y = g[1]
#x = x+ 9
#y = 10 - y
# start ( y , x)
    s_y= int(10 - s_y) 
    s_x= int(s_x+9)
    
    g_x= int(g_x+9)
    g_y= int(10 - g_y)
    start = (s_y,s_x)
    goal = (g_y,g_x)
    print start
    print goal
    print "inside function"
    Neighbors_list = [(0,1),(0,-1),(1,0),(-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:Distance_cost(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            global path_goal2start,path_start2goal
	    path_goal2start = []
	    path_start2goal = []
            while current in came_from:
                path_goal2start.append(current)
                current = came_from[current]
	    n = len(path_goal2start)-1
	    while n>=0:
		path_start2goal.append(path_goal2start[n])
		n = n-1
            return path_start2goal

        close_set.add(current)
        for i, j in Neighbors_list:
            PickNeighbor = current[0] + i, current[1] + j            
            tentativeg_score = gscore[current] + Distance_cost(current, PickNeighbor)
            if 0 <= PickNeighbor[0] < array.shape[0]:
                if 0 <= PickNeighbor[1] < array.shape[1]:                
                    if array[PickNeighbor[0]][PickNeighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue
                
            if PickNeighbor in close_set and tentativeg_score >= gscore.get(PickNeighbor, 0):
                continue
                
            if  tentativeg_score < gscore.get(PickNeighbor, 0) or PickNeighbor not in [i[1]for i in oheap]:
                came_from[PickNeighbor] = current
                gscore[PickNeighbor] = tentativeg_score
                fscore[PickNeighbor] = tentativeg_score + Distance_cost(PickNeighbor, goal)
                heappush(oheap, (fscore[PickNeighbor], PickNeighbor))
                
    return False

def movement():
	global flag_r,flag_t,path_start2goal,temp_pos,Finalloc_x,Finalloc_y,ctr,path_start2goal,path_goal2start
	temp_pos = path_start2goal.pop(0)
        Finalloc_y= 10 - temp_pos[0]
        Finalloc_x = temp_pos[1] - 9
	rospy.Subscriber("base_pose_ground_truth", Odometry, BotMoveFunction)


def BotMoveFunction(odom):
    	global flag_r,flag_t,path_start2goal,temp_pos,Finalloc_x,Finalloc_y,ctr,flag,PathMap,goal_x,goal_y,currentpos_x,currentpos_y

	publish_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=100)
       
	if flag_r == 1 and flag_t ==1:		
		flag_r=0
		flag_t=0
		#print "goal_x =" 
		#print goal_x
		#print "goal_y ="
		#print goal_y
                x_ch = rospy.get_param('goalx')
		#print"x_ch = "
		#print x_ch
		y_ch = rospy.get_param('goaly')		
		#print"y_ch = "
		#print y_ch
		if len(path_start2goal) ==0:
		    print "Reached Destination: You have 10 seconds to give new path"
		    time.sleep(5)
		    print "Exiting SleeP"
		    if goal_x != x_ch or goal_y != y_ch:
			goal_x = x_ch
			goal_y = y_ch
			print "goal changed"
			#print "CURRENT"
			#print currentpos_x
			#print currentpos_y
			print Astar_Algo(PathMap,(currentpos_x,currentpos_y),(x_ch,y_ch))	
			movement()
		    return
		#if goal_x != 1.0 and goal_y != 13.0:
		
		#	print "goal changed"
		 #       main()
		
		if goal_x != x_ch or goal_y != y_ch:
			goal_x = x_ch
			#flag = 1
#		    if goal_y != y_ch and flag ==1:
			goal_y = y_ch
			print "Goal changed"
			#flag = 0			
			#path_goal2start = []
			#path_start2goal = []
#			current_x = odom.pose.pose.position.x
#			cur_y = odom.pose.pose.position.y
			#print "CURRENT"
			#print currentpos_x
			#print currentpos_y
			
			print Astar_Algo(PathMap,(currentpos_x,currentpos_y),(x_ch,y_ch))	
			movement()
			#rospy.Subscriber("base_pose_ground_truth", Odometry, BotMoveFunction)
			#return
			
					
		temp_pos = path_start2goal.pop(0)
		Finalloc_y= 10 - temp_pos[0] - .45
		Finalloc_x = temp_pos[1] - 9 + .45
	currentpos_x=odom.pose.pose.position.x
	currentpos_y=odom.pose.pose.position.y
	quaternion=(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w)
	euler = euler_from_quaternion(quaternion)
	orient_robot= euler[2]

	#print Finalloc_x
	#print Finalloc_y
	if Finalloc_x==currentpos_x:
		if Finalloc_y>currentpos_y:
			orient_goal=math.pi/2
		elif Finalloc_y<currentpos_y:
			orient_goal=-(math.pi/2)
		else:
			orient_goal=orient_robot
	else:
		orient_goal=math.atan2((Finalloc_y-currentpos_y),(Finalloc_x-currentpos_x))
	if orient_goal-orient_robot>0.1 and flag_r==0:
		msg=Twist()
		msg.linear.x=0
		msg.linear.y=0
		msg.linear.z=0
		msg.angular.z=+0.75
		publish_vel.publish(msg)
		return
	if orient_goal-orient_robot<-0.1 and flag_r==0:
		msg=Twist()
		msg.linear.x=0
		msg.linear.y=0
		msg.linear.z=0
		msg.angular.z=-0.75
		publish_vel.publish(msg)
		return
	else:
		flag_r=1
	if abs(Finalloc_x-currentpos_x)>0.15 or abs(Finalloc_y-currentpos_y)>0.15 and flag_t==0:
		msg1=Twist()
		msg1.linear.x=3.0
		msg1.linear.y=0
		msg1.linear.z=0
		publish_vel.publish(msg1)
		return
	else:
		flag_t=1

def main():
    
    global path_start2goal,temp_pos,Finalloc_x,Finalloc_y,goal_x,goal_y,startx,starty,PathMap
    goalx= rospy.get_param('goalx')
    goaly = rospy.get_param('goaly')
   # print x_up 
   # print y_up
   # if x_up == -50.0 and y_up == -50.0:
   #     goal_x = 1.0
   #     goal_y = 13.0
   # if x_up != -50.0 or y_up != -50.0:
   #         goal_x = x_up
   #     if y_up != -50.0:
   #         goal_y = y_up
   #         ctr = 1
   
    print Astar_Algo(PathMap, (startx,starty), (4.5,9.0))
    movement()
     
    publish_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=100)
    rospy.spin()



if __name__ == "__main__":
    rospy.init_node('lab5')
    main()
