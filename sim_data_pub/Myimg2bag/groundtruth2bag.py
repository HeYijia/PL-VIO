import time, sys, os
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image,Imu
from geometry_msgs.msg import Vector3

def CompSortFileNamesNr(f):
    g = os.path.splitext(os.path.split(f)[1])[0] #get the file of the
    numbertext = ''.join(c for c in g if c.isdigit())
    return int(numbertext)

def ReadGT(filename):
    '''return ground truth pose and timestamp with TUM style'''
    file = open(filename,'r')
    all = file.readlines()
    timestamp = []
    imu_data = []
    for f in all:
        s = f.rstrip('\n')
#	print s
#	print s.split()
        s = ' '.join(s.split());
        line = s.split(' ')
	print line
        timestamp.append(line[0])
        imu_data.append(line[1:])
    return timestamp,imu_data


def CreateBag(args):#img,imu, bagname, timestamps
    '''read time stamps'''
    imutimesteps,imudata = ReadGT(args[1]) #the url of  IMU data
    if not os.path.exists(args[2]):
	os.system(r'touch %s' % args[2])
    bag = rosbag.Bag(args[2], 'w')

    try:
        for i in range(len(imudata)):
            imu = Imu()
            angular_v = Vector3()
            linear_a = Vector3()
            angular_v.x = float(imudata[i][3])
            angular_v.y = float(imudata[i][4])
            angular_v.z = float(imudata[i][5])
            linear_a.x = float(imudata[i][0])
            linear_a.y = float(imudata[i][1])
            linear_a.z = float(imudata[i][2])
            imuStamp = rospy.rostime.Time.from_sec(float(imutimesteps[i]))
            imu.header.stamp=imuStamp
            imu.angular_velocity = angular_v
            imu.linear_acceleration = linear_a

            bag.write("IMU/imu_raw",imu,imuStamp)
    finally:
        bag.close()



if __name__ == "__main__":
      print(sys.argv)
      CreateBag(sys.argv[1:])
