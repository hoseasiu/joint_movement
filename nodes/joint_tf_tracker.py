#!/usr/bin/env python  
import roslib
roslib.load_manifest('joint_movement')
import rospy
import math
import tf
#from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from collections import deque
import xlwt
from numpy import *

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
#        print i, "elements length:", len(elements), "deque length:", len(deque[0])
        deque[0][i].append(elements[i])

    return deque

# find the number label of each person the kinect(s) see
# takes the output of listener.getFrameStrings(), a list of frames as strings
def getSkeletonList(tfFrames):
    skeletonList = []
    for frame in tfFrames:
        # find every frame with the label "torso" and return the number in the name "torso_#"
        if (frame.find("torso") != -1):
            skeletonList.append(int(frame.split("_")[1]))
    if (len(skeletonList) == 1):
        print "Found", str(len(skeletonList)), "person"
    else:
        print "Found", str(len(skeletonList)), "people"
    return skeletonList
    

if __name__ == '__main__':
    rospy.init_node('joint_tf_tracker')		            # start the node
    pub = rospy.Publisher('joint_state', String)		# start the rostopic to be published to
    listener = tf.TransformListener()                   # start the transform listener

    velocityTfBroadcasters = []                         # vector of transforms that represent users' velocities
    orientationTfBroadcasters = []                      # vector of transforms that represent users' orientations

    prevSkeletonList = []                               # tracks users that are entering/leaving frame to create position and rotation arrays appropriately
    
    prevTorsoPos = []                                   # stores 3-element vectors of previous positions for instantaneous velocity calculations
    currentTorsoPosDeques = []                          # stores 3-element tuples (each element is a deque of up to 10 elements) of positions for smoothed positions (arrays of x, y and z)
    currentOrientationDeques = []                          # stores 4-element tuples (each element is a deque of up to 10 elements) of rotations for smoothed positions (arrays of qx, qy, qz, and qw)

    timeWindow = 1                                      # seconds
    windowSize = timeWindow*10                          # 10 frames per second

    while not rospy.is_shutdown():
        try:
            skeletonList = getSkeletonList(listener.getFrameStrings())
            
            # TODO - assumes that the skeletonList never loses members, only gains them... valid assumption? (if it is, that probably needs to be changed)
            # TODO - scan over the currentTorsoPosDeques and currentOrientationDeques to remove skeletons that have stopped moving entirely (not actually present anymore)
                # maybe use a count of some kind so that they are removed only after ~3-5 seconds of nonactivity instead of 1?
            
            for s in skeletonList:
                if s not in prevSkeletonList:
                    prevTorsoPos.append([0,0,0])
                    currentTorsoPosDeques.append((deque([]), deque([]), deque([])))                         # tuple of 3 deques for x, y, and z positions
                    currentOrientationDeques.append((deque([]), deque([]), deque([])))                      # tuple of 3 deques for rpy rotations
                    velocityTfBroadcasters.append(tf.TransformBroadcaster())                                # add a new broadcaster for representing velocities
                    orientationTfBroadcasters.append(tf.TransformBroadcaster())                             # add a new broadcaster for representing orientations

                # position in the list does NOT necessarily correspond to the skeleton number
                sIndex = skeletonList.index(s)
                
                (transTorso,rotTorso) = listener.lookupTransform('/openni_depth_frame', '/torso_'+str(s), rospy.Time(0))

                # smooth out the position data
                addToMovingWindow(transTorso, windowSize, currentTorsoPosDeques[sIndex])
                posSmooth = [average(currentTorsoPosDeques[sIndex][0]), average(currentTorsoPosDeques[sIndex][1]), average(currentTorsoPosDeques[sIndex][2])]
              	
              	# calculate the velocity
                velSmooth = [posSmooth[0]-prevTorsoPos[sIndex][0], posSmooth[1]-prevTorsoPos[sIndex][1], posSmooth[2]-prevTorsoPos[sIndex][2]]

                # average the orientations of the shoulders and the hips                
                (transNeck,rotNeck) = listener.lookupTransform('/openni_depth_frame', '/neck_'+str(skeletonList[0]), rospy.Time(0))                                 # neck
                (transLeftShoulder,rotLeftShoulder) = listener.lookupTransform('/openni_depth_frame', '/left_shoulder_'+str(skeletonList[0]), rospy.Time(0))        # left shoulder
                (transRightShoulder,rotRightShoulder) = listener.lookupTransform('/openni_depth_frame', '/right_shoulder_'+str(skeletonList[0]), rospy.Time(0))     # right shoulder
                (transLeftHip,rotLeftHip) = listener.lookupTransform('/openni_depth_frame', '/left_hip_'+str(skeletonList[0]), rospy.Time(0))                       # left hip
                (transRightHip,rotRightHip) = listener.lookupTransform('/openni_depth_frame', '/right_hip_'+str(skeletonList[0]), rospy.Time(0))                    # right hip
                
                tVec = matrix(transNeck)-matrix(transTorso)                     # torso vector (torso to neck)
                sVec = matrix(transRightShoulder)-matrix(transLeftShoulder)     # shoulder vector (left to right)
                hVec = matrix(transRightHip)-matrix(transLeftHip)               # hip vector (left to right)
                
                oVec = (cross(tVec,sVec)+cross(tVec,hVec))                      # sum of (tVec cross sVec) and (tVec cross hVec)
                oVec = oVec/linalg.norm(oVec)                                   # divide by the norm to get the unit vector (orientation is always a unit vector)
                oVec = oVec[0]                                                  # change from matrix to vector

                print "Skeleton ID:", s
                print "position", posSmooth
                print "velocity", velSmooth
                print "orientation", oVec
                print "----------------------------- \n"

                # visualization section ######################
                # broadcast transforms for visualization (can comment out later for speed)
                
                oVecScaled = [x*5 for x in oVec]                         # scale the orientation so it's easier to see on rviz
                velSmoothScaled = [x*10 for x in velSmooth]              # scale the velocity vector so it's easier to see in rviz
                
                velocityTfBroadcasters[sIndex].sendTransform([velSmoothScaled[1], velSmoothScaled[2], velSmoothScaled[0]], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "velocity_"+str(s), "torso_"+str(s))
                
                orientationTfBroadcasters[sIndex].sendTransform([0,0,-1], tf.transformations.quaternion_from_euler(oVecScaled[0],oVecScaled[1],oVecScaled[2]), rospy.Time.now(), "orientation_"+str(s), "torso_"+str(s))

                ###############################################

                # smooth out the orientation data
                addToMovingWindow(oVec, windowSize, currentOrientationDeques[sIndex])
                oVecSmooth = [average(currentOrientationDeques[sIndex][0]), average(currentOrientationDeques[sIndex][1]), average(currentOrientationDeques[sIndex][2])]
                
                # TODO - correct this for new format
                dataLog = str(s), str(posSmooth[0]), str(posSmooth[1]), str(posSmooth[2]), str(velSmooth[0]), str(velSmooth[1]), str(velSmooth[2]), str(oVecSmooth[0]), str(oVecSmooth[1]), str(oVecSmooth[2])
#                print dataLog
#                rospy.loginfo(dataLog)
#                pub.publish(String(dataLog))
               
                prevTorsoPos[sIndex] = posSmooth
            
            # put any new skeletons into the old skeleton list
            prevSkeletonList = skeletonList
            rospy.Rate(10.0).sleep()            # checking at 10 hz
          	  
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "I'm not getting anything from the transform. Did you start openni_tracker?"
            rospy.Rate(1.0).sleep()             # checks every second to see if the data is coming through
            continue
