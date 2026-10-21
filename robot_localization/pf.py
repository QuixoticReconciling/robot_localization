#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))


class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """
    def __init__(self):
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 3000          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update


        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(ParticleCloud, "particle_cloud", qos_profile_sensor_data)

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            self.update_particles_with_laser(r, theta)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose based on particles
            self.resample_particles()               # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_robot_pose(self):
        """ Updates the robot's pose estimate from the current particle cloud.
            Normalizes the particle weighted mean of the particles to produce 
            a single pose estimate. The pose is then stored and used with the 
            odom pose to update the map.
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        #what the code below is going to do is for x and y is just weight the x, y contribution of particles by their weight attr then sum
        #for theta it can't just weight and sum because angles wrap around, so we sum x, y components of sin, cos theta to get a vector
        #then we find the angle of this vector through atan2
        new_x = sum(p.x*p.w for p in self.particle_cloud)# /self.n_particles
        new_y = sum(p.y*p.w for p in self.particle_cloud)# /self.n_particles
        new_theta_y = sum(math.sin(p.theta) * p.w for p in self.particle_cloud)  #/self.n_particles
        new_theta_x = sum(math.cos(p.theta) * p.w for p in self.particle_cloud) #/self.n_particles <-- I realized it wasn't working bevause I was taking mean after weighting
        new_theta = math.atan2(new_theta_y, new_theta_x) #use atan2 to make sure it works outside of just q1 and q4
        
        #we make the new x, y, theta into a particle to get out a pose efficienty and assign it to the robot pose
        new_particle = Particle(x=new_x, y = new_y, theta = new_theta, w=1.0)
        self.robot_pose = new_particle.as_pose()
        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                            self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")

    def update_particles_with_odom(self):   
        """ Updates the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry. The
            delta value is then used to calculate the magnitude and the direction of
            the displacement vector. the displacement vector is used to update the
            position of the actual particles.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # compute the change in x,y,theta since our last update. This is the displacement vector from our last state
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            #if we dont have a previous value the next previous value will become the new_pos we computed that we didn't use
            self.current_odom_xy_theta = new_odom_xy_theta
            return
        # note to self: here is what we know right now, we know how much odom says the x, y, and angle have changed 
        # but the problem is to change the particles we need to know how much each change has happened relative to the robot
        # because each particle must change by that much relative to itself, not relative to odom
        
        #we did a bunch of sketching to get the logic below: 
        #we calculate the magnitude and direction of the displacement vector and then rotate it onto each particle
        #this give us a vector in the odom frame that we can just add to the particle to translate it and then we just add 
        #the change in theta onto it as well
        dist = math.sqrt(delta[0]**2 + delta[1]**2) # magnitude
        direction = math.atan2(delta[1], delta[0])
        
        # Modifying the actual particles
        for p in self.particle_cloud: #the logic behind this update is going to be explained in a sketch we did that will be on the github, took us a while to get this
            p_theta_diff = p.theta-old_odom_xy_theta[2]
            p.x += dist * math.cos(direction+ p_theta_diff)
            p.y += dist * math.sin(direction + p_theta_diff)
            p.theta += delta[2]

    def resample_particles(self):
        """ Resamples the particles according to the new particle weights.
            This function first normalizes the particles so the weights sum
            to one before taking the cumulative sum of the weights that divides
            the range into n_particles slices. 'searchsorted' is used to find
            which slice each new, randomly generated value falls into. Noise is
            added to prevent particles from collapsing onto a single point.
        """
        # make sure the distribution is normalized
        self.normalize_particles()
        rng = np.random.default_rng()
        cumulative_weights = np.cumsum([p.w for p in self.particle_cloud])
        cumulative_weights[-1] = 1.0
        indices = np.searchsorted(cumulative_weights, rng.random(self.n_particles))
        noise_x = rng.normal(0.0, 0.2, self.n_particles)
        noise_y = rng.normal(0.0, 0.2, self.n_particles)
        noise_theta = rng.normal(0.0, math.pi / 20, self.n_particles)
        weight = 1.0 / self.n_particles
        source = self.particle_cloud
        self.particle_cloud = [
            Particle(source[i].x + dx, source[i].y + dy,
                     source[i].theta + dtheta, weight)
            for i, dx, dy, dtheta in zip(indices, noise_x, noise_y, noise_theta)
        ]

    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data.
            The function takes in the distance 'r' and heading 'theta'
            of 8 different lidar scans. It takes the 'r' and 'theta'
            values and moves the x and y position of the particle cloud
            based on those values. It then uses the
            'get_closest_obstacle_distance' function to find the closest
            distance for any given particle. Those distances for each check
            are added up into an error, then normalized based on all the
            particle errors.
        """
        # This bag has 720 readings:
        assert len(r) == len(theta) == 720
        assert abs((theta[90] - theta[0]) - math.pi / 4) < 1.5 * math.pi / 180
        #we pick these lidar readings because all of their angles are offset 45 degrees from one another
        checks = [r[0], r[90], r[180], r[270], r[360], r[450], r[540], r[630]]
        check_thetas = [theta[0], theta[90], theta[180], theta[270],
                        theta[360], theta[450], theta[540], theta[630]]

        #we clean out lidar readings that go to posive infinity or negatiev infinity
        for i in range(8):
            if checks[i] > 100:
                checks[i] = 10
            if checks[i] < 0:
                checks[i] = 0

        for p in self.particle_cloud:
            closest_distances = []
            for i in range(8):
                #for every lidar scan we add it onto the x y of the point and check for walls
                x = p.x + checks[i] * math.cos(check_thetas[i] + p.theta)
                y = p.y + checks[i] * math.sin(check_thetas[i] + p.theta)
                closest_distances.append(self.occupancy_field.get_closest_obstacle_distance(x, y))

            if all(math.isfinite(distance) for distance in closest_distances):
                #the error is the displacement from walls for each of the scans summed up, the perfect 
                #particle will have an error of 0 because all lidar scans for each angle will be 0 dist from a wall
                error = abs(0 - sum(closest_distances))
                #we calculate a gaussian kernel so low scores get high weights and high scores get low weights
                p.w = 1e-6 + math.exp(-0.5 * (error/.5) ** 2)
            else:
                p.w = 1e-6


        
        

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            This function creates a particle cloud using a normal 
            distribution centered around 0. The standard deviation
            for x and y is 10m and for theta it's pi radians or 180
            degrees.
        """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        self.particle_cloud = []
        #we cast a really wide net with large standard deviations for x, y theta because we are solving the kidnapped robot problem
        #where we have no initial guess for the robot
        rng = np.random.default_rng()
        std_x = 10
        std_y = 10
        std_theta = math.pi
        
        #we need both noise and random positions for x, y, and theta
        #if we just used random posiitons the mean would come back to 0, or if we just added noise to the initial position
        #its mean would still be the initial position
        noise_x = rng.normal(loc=0.0, scale=std_x, size=self.n_particles)
        noise_y = rng.normal(loc=0.0, scale=std_y, size=self.n_particles)
        noise_theta = rng.normal(loc=0.0, scale=std_theta, size=self.n_particles)
        random_x = rng.normal(loc=0.0, scale=std_x, size=self.n_particles)
        random_y = rng.normal(loc=0.0, scale=std_y, size=self.n_particles)
        random_theta = rng.normal(loc=0.0, scale=std_theta, size=self.n_particles)
        for i in range(self.n_particles):
            x = random_x[i] + noise_x[i]
            y = random_y[i] + noise_y[i]
            theta = random_theta[i] + noise_theta[i]    
            theta = math.atan2(math.sin(theta), math.cos(theta)) # bounds theta between -pi and pi
            self.particle_cloud.append(Particle(x, y, theta, w = 1.0))

        self.normalize_particles()
        self.update_robot_pose()

    def normalize_particles(self):
        """ Normalizes the particle weights so they all sum to one. """
        total = 0
        #keep a total sum of all the weight values
        for p in self.particle_cloud:
            total += p.w
        total_check = 0
        #now normalize the weights by making them the percentage they make of the total weights
        #also to check that all the weights = 1 after we sum up their values into total_check
        for p in self.particle_cloud:
            p.w = p.w/total
            total_check += p.w

        assert(abs(total_check-1) < 10e-6)

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg

def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
