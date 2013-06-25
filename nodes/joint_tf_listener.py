#!/usr/bin/env python  
import roslib
roslib.load_manifest('joint_movement')
import rospy
import math
import tf
from std_msgs.msg import String
from collections import deque
import xlwt

# take a simple average
def average(numList):
    sum = 0
    for i in numList:
        sum += i
    if len(numList) > 0:
        return sum/len(numList)
    else:
        return 0

# add each element of a 3-element array to a specified deque
# assumes all deques are of the same size
def addToMovingWindow(elements, windowSize, *deque):
    if len(deque[0][0]) > windowSize-1:         # remove oldest element from the deques if bigger than a certain windowSize
        for i in deque[0]:
            i.popleft()

    for i in range(len(deque[0])):
        deque[0][i].append(elements[i])

    return deque

# write the data to an excel spreadsheet
# data is in the following order: x, y, z, xDot, yDot, zDot, qx, qy, qz, qw
def writeData(position, velocity, rotation, counter):
    for i in range(len(position)):
        worksheet.write(counter, i, position[i])
        worksheet.write(counter, i+3, velocity[i])
    for i in range(len(rotation)):
        worksheet.write(counter, i+6, rotation[i])
    workbook.save('torsoData.xls')

if __name__ == '__main__':
    rospy.init_node('tf_torso')		            # start the node
    pub = rospy.Publisher('joint_state', String)		# start the rostopic to be published to
    listener = tf.TransformListener()           # start the transform listener
    
    prevPos = [0,0,0]           # for calculating velocities
    (xArray, yArray, zArray) = (deque([]), deque([]), deque([]))                            # position
    (qxArray, qyArray, qzArray, qwArray) = (deque([]), deque([]), deque([]), deque([]))         # rotation
    
    windowSize = 10

    # initialize data logging stuff
    workbook = xlwt.Workbook(encoding = 'ascii')
    worksheet = workbook.add_sheet('Xtion data')
    dataCounter = 0

    while not rospy.is_shutdown():
        try:
            print listener.getFrameStrings()
            # track the torso relative to the camera- TODO - can also track any other joint
            (trans,rot) = listener.lookupTransform('/openni_depth_frame', '/torso_3', rospy.Time(0))
                        
            # smooth out the position data
            addToMovingWindow(trans, windowSize, (xArray, yArray, zArray))
            posSmooth = [average(xArray), average(yArray), average(zArray)]
          	
          	# calculate the velocity
            velSmooth = [posSmooth[0]-prevPos[0], posSmooth[1]-prevPos[1], posSmooth[2]-prevPos[2]]
                      	
            # smooth out the rotation data
            addToMovingWindow(rot, windowSize, (qxArray, qyArray, qzArray, qwArray))
            rotSmooth = [average(qxArray), average(qyArray), average(qzArray), average(qwArray)]
            
            # write the data to file
            writeData(posSmooth, velSmooth, rotSmooth, dataCounter)
            dataCounter+=1

            dataLog = str(posSmooth[0]) + "," + str(posSmooth[1]) + "," + str(posSmooth[2]) + "," + str(velSmooth[0]) + "," + str(velSmooth[1]) + "," + str(velSmooth[2]) + "," + str(rotSmooth[0]) + str(rotSmooth[1]) + "," + str(rotSmooth[2]) + "," + str(rotSmooth[3])
            rospy.loginfo(dataLog)
            pub.publish(String(dataLog))
            
            prevPos = posSmooth

            rospy.Rate(10.0).sleep()            # checking at 10 hz
          	  
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "I'm not getting anything from the transform"
            rospy.Rate(1.0).sleep()             # checks every second to see if the data is coming through
            continue
