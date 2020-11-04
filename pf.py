from geometry_msgs.msg import Pose, PoseArray, Quaternion, Pose2D, Twist
from pf_base import PFLocaliserBase
import math
import rospy
import util
from scipy import stats
import numpy as np

from util import rotateQuaternion, getHeading
import random

from time import time
from sensor_msgs.msg import LaserScan

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
 	self.ODOM_ROTATION_NOISE = 2 * math.pi
	self.ODOM_TRANSLATION_NOISE = 0.1
	self.ODOM_DRIFT_NOISE = 0.1
	self.particle_weights = []
	self.twist = Twist()
	self.w_slow = 0.06
	self.A_SLOW = 0.15
	self.w_fast = 0.01
	self.A_FAST = 0.35

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 10     # Number of readings to predict
        self.MOVEMENT_RANGE = 0.7     # Number of readings to predict
        self.ANGLE_RANGE = 2.0     # Number of readings to predict
        self.NUMBER_OF_PARTICLES = 200
        self.MIN_VELOCITY = 0.1
	self.last_time = rospy.Time.now()
	self.scan = LaserScan
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        SPREAD = 10
        result = PoseArray()
	self.weights = []
        
        for i in range(self.NUMBER_OF_PARTICLES):
            new_point = Pose()
            new_point.position.x = initialpose.pose.pose.position.x + random.uniform(-SPREAD, SPREAD)
            new_point.position.y = initialpose.pose.pose.position.y + random.uniform(-SPREAD, SPREAD)

            new_point.orientation = util.rotateQuaternion(initialpose.pose.pose.orientation, random.uniform(-math.pi, math.pi))
            result.poses.append(new_point)
	    self.weights.append(1.0)
        return result

    def generate_random_map_pose(self):
	pose = Pose()
	pose.position.x = self.sensor_model.map_origin_x + random.uniform(-(self.sensor_model.map_width / 2.0) * self.sensor_model.map_resolution,(self.sensor_model.map_width / 2.0) * self.sensor_model.map_resolution)
	pose.position.y = self.sensor_model.map_origin_y + random.uniform(-(self.sensor_model.map_height / 2.0) * self.sensor_model.map_resolution, (self.sensor_model.map_height / 2.0) * self.sensor_model.map_resolution)
	thetad = random.uniform(-math.pi, math.pi)
	pose.position.z = 0
	pose.orientation = util.rotateQuaternion(Quaternion(w=1.0),  thetad)
	return pose

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
	self.scan = scan
	movement_particles = []

        w = self.twist.angular.z
        v = self.twist.linear.x
	self.twist = Twist()
	wv = math.sqrt(v * v + w * w)

	dtime = rospy.Time.now().to_sec() - self.last_time.to_sec()
	self.last_time = rospy.Time.now()

        for i in self.particlecloud.poses:
		x = i.position.x
		y = i.position.y
		theta = util.getHeading(i.orientation)
		for j in range(4):
			p = Pose()

			p.position.x = x + random.uniform(-0.2,0.2)
			p.position.y = y + random.uniform(-0.2,0.2)
			p.position.z = 0

			thetad = theta + random.uniform(-0.4,0.4)
			p.orientation = util.rotateQuaternion(Quaternion(w=1.0), thetad)
			
			probn = self.sensor_model.get_weight(scan,p) ** 2
			movement_particles.append((probn, p))
	w_avg = 0.0
	for x in movement_particles:
		w_avg += x[0] / len(movement_particles)
	
	self.w_slow += self.A_SLOW * (w_avg - self.w_slow)
	self.w_fast += self.A_FAST * (w_avg - self.w_fast)

	particlecloudnew = PoseArray()
        weights = np.asarray([i[0] for i in movement_particles]).cumsum()

	
	new_weights = []
	for i in range(self.NUMBER_OF_PARTICLES):
		j = random.random() * w_avg * len(movement_particles)
		index = np.searchsorted(weights, j)		
		pose = movement_particles[index][1]
		weight = movement_particles[index][0]
		if random.random() > (self.w_fast / self.w_slow):
			pose = self.generate_random_map_pose()
			weight = 0.0
			
		particlecloudnew.poses.append(pose)
		new_weights.append(weight)
	#if wv > 0.03: #Only update if movement is occured to avoid focusing on slightly higher poses
	self.particlecloud.poses = particlecloudnew.poses
	self.weights = new_weights

    def weighted_zscore(self):

	result = Pose()
        sin_rotation = 0.0
        cos_rotation = 0.0
	total = 0.0
        for i in range(self.NUMBER_OF_PARTICLES):
	    total += self.weights[i]
	j = 0
        for i in self.particlecloud.poses:
            result.position.x += i.position.x * self.weights[j] / total
            result.position.y += i.position.y * self.weights[j] / total
            theta = util.getHeading(i.orientation) * self.weights[j] /total
            sin_rotation += math.sin(theta) * self.weights[j] / total
            cos_rotation += math.cos(theta) * self.weights[j] / total
	    j += 1
        rotation = math.atan2(sin_rotation, cos_rotation)
        result.orientation = util.rotateQuaternion(Quaternion(w=1.0), rotation)
	dist = []
        for i in self.particlecloud.poses:
		x = (i.position.x - result.position.x) ** 2 + (i.position.y - result.position.y) ** 2
		dist.append(math.sqrt(x))
	dist = np.array(dist)
	zscore = stats.zscore(dist)
	result = Pose()
        sin_rotation = 0.0
        cos_rotation = 0.0
	total = 0.0
	#print(zscore)
        for i in range(self.NUMBER_OF_PARTICLES):
		if zscore[i] < 0.7 and zscore[i] > -0.7:
       		    result.position.x += self.particlecloud.poses[i].position.x
        	    result.position.y += self.particlecloud.poses[i].position.y
        	    theta = util.getHeading(self.particlecloud.poses[i].orientation)
        	    sin_rotation += math.sin(theta)
        	    cos_rotation += math.cos(theta)
	 	    total += 1.0
	print(self.NUMBER_OF_PARTICLES)
	print(total)
	result.position.x /= total
	result.position.y /= total
        rotation = math.atan2(sin_rotation / total, cos_rotation / total)
        result.orientation = util.rotateQuaternion(Quaternion(w=1.0), rotation)
	
        return result

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        return self.weighted_zscore()

