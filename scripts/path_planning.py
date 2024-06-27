#!/usr/bin/env python3
from fsd_path_planning.utils.math_utils import unit_2d_vector_from_angle, rotate
from fsd_path_planning.utils.cone_types import ConeTypes
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
import rospy
import numpy as np
from numpy import random
import math
import timeit
from std_msgs.msg import String, Int16, Float32
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry, Path
from fs_msgs.msg import PlannedPath
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import threading
from copy import deepcopy
import time
from scipy.interpolate import splprep, splev
import pandas as pd

# Rest of the code

UNCOLORED_CONES = True # Set to True to use uncolored cones
VERBOSE = True # Set to True to generate the colored path without the false cones
planner = PathPlanner(MissionTypes.trackdrive)

### for filtering out duplicate cones ###
seen_left_cones = set() 
seen_right_cones = set()

class PlannerNode:
    def __init__(self):
        """initialize the path planner node"""
        rospy.init_node("path_planner", anonymous=True)
        rospy.Subscriber("/slam/output/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/slam/output/markers_map", MarkerArray, self.marker_callback)
        rospy.Subscriber( "/navigation/speed_profiler/path", PlannedPath, self.curr_path_callback)

        self.recorded_path_pub = rospy.Publisher("/recorded_path", Path, queue_size=10)
        self.original_path_pub = rospy.Publisher("/original_path", Path, queue_size=10)
        self.generated_path_pub = rospy.Publisher("/generated_path", Path, queue_size=10)
        self.generate_heading_pub = rospy.Publisher("/heading", Marker, queue_size=10)
        self.uncolored_path_pub = rospy.Publisher("/uncolored_path", Path, queue_size=10)
        self.false_cones_pub = rospy.Publisher("/false_cones", MarkerArray, queue_size=10)
        self.yaw_test = rospy.Publisher("/yaw_test", Float32, queue_size=10)
        rospy.Timer(rospy.Duration(1/2), self.customPath_to_path)
        rospy.Timer(rospy.Duration(1/2), self.record_path)
        self.br = tf.TransformBroadcaster()
        

        self.curr_path = None
        self.recorded_node = None
        
        self.stamp = None
        self.cones_right_raw = np.array([])
        self.cones_left_raw = np.array([])
        self.false_cones = MarkerArray()
        self.false_p_left = MarkerArray()
        self.rate = rospy.Rate(100)
        self.car_position = None
        self.car_direction = None
        self.new_path = Path()
        self.recorded_path = Path()
        self.recorded_path.header.frame_id = "odom"
        self.aligned_car_pos = None
        print("running")



    def run_node(self):
        """run the path planner node"""
        while not rospy.is_shutdown():
            if self.car_position is not None and self.car_direction is not None and self.cones_left_raw.size != 0 and self.cones_right_raw.size != 0:
                
                start_time = time.time()
                self.generate_new_path(self.car_position, self.car_direction, self.cones_left_raw, self.cones_right_raw, both_cones=True)
                end_time = time.time()
                elapsed_time = end_time - start_time
                print(f"Time taken: {elapsed_time} seconds")
                self.false_cones_pub.publish(self.false_cones)
            self.rate.sleep()

    def record_path(self, event):
        """record path from by aligning the car position to the closest point in generated path,
        apply spline to the sets of points and interpolate into a new path message"""

        if self.aligned_car_pos is not None:
            self.recorded_path.poses = []
            if self.recorded_node is None:
                self.recorded_node = self.aligned_car_pos
            else:
                self.recorded_node = np.vstack((self.recorded_node, self.aligned_car_pos))
                self.recorded_node = np.unique(self.recorded_node, axis=0)
                x = self.recorded_node[:,0]
                y = self.recorded_node[:,1]
                if len(self.recorded_node) > 3:
                    tck, u = splprep([x, y], s=0)

                    # Create new points from the spline
                    u_new = np.linspace(0, 1, 10*len(self.recorded_node))
                    x_new, y_new = splev(u_new, tck)
                    # print(x_new, y_new)
                    for i in range(len(x_new)):
                        new_pose = PoseStamped()
                        new_pose.header.frame_id = "odom"
                        new_pose.header.stamp = self.stamp
                        new_pose.pose.position.x = x_new[i]
                        new_pose.pose.position.y = y_new[i]
                        self.recorded_path.poses.append(new_pose)
                    self.recorded_path_pub.publish(self.recorded_path)




    def align_car_pos(self):
        """
        take the current car position and find the closest points in the generated path. append the point to be the recorded path.
        used variables:
        - self.new_path (n x 2): the generated path
        - self.car_position ([x,y]): the car's position        
        """

        num_points = 5 # the number of points iterate through in the generated path
        closest_dist = np.inf
        closest_point = None
        for point  in self.new_path.poses[:num_points]:
            point = np.array([point.pose.position.x, point.pose.position.y])
            dist = math.dist(self.car_position, point)
            if dist < closest_dist:
                closest_dist = dist
                closest_point = point
        return closest_point





    def generate_new_path(self, car_pos, car_dir,  cones_left_raw, cones_right_raw, both_cones = True):
        """
        Generate and publish a new path based on the car's position, direction and the cones detected by the camera.
        Args:
        - car_pos ([x,y]): The car's position in odometry frame.
        - car_dir ([x,y]): The car's 2d vector from yaw in odometry frame.
        - cones_left_raw (n x 2): The cones detected on the left side of the car.
        - cones_right_raw (n x 2): The cones detected on the right side of the car.
        - both_cones (bool): If False, only the left cones will be used.

        """

        if not cones_left_raw.size:
            return
        
        cones_left_raw, cones_right_raw = self.filter_markers(cones_left_raw, cones_right_raw)
        
        mask_is_left = np.ones(len(cones_left_raw), dtype=bool)
        mask_is_right = np.ones(len(cones_right_raw), dtype=bool)

        aligned_car_pos = self.align_car_pos()
        if aligned_car_pos is not None and math.dist(aligned_car_pos, car_pos) < 1.5:
            self.aligned_car_pos = aligned_car_pos
            print("aligned")


        cones_left_adjusted = cones_left_raw - car_pos
        cones_right_adjusted = cones_right_raw - car_pos

        mask_is_left[np.argsort(np.linalg.norm(cones_left_adjusted, axis=1))[5:]] = False
        mask_is_right[np.argsort(np.linalg.norm(cones_right_adjusted, axis=1))[5:]] = False
       
        if VERBOSE:
            combined_cones = np.vstack((cones_left_raw, cones_right_raw))
            df = pd.DataFrame(combined_cones, columns=['x', 'y'])
            
            # Find duplicates
            duplicate_cones = df[df.duplicated()]
            if duplicate_cones.size != 0:
                print("Duplicate cones found")

            
            path = self.run_path_planner(cones_left_raw, cones_right_raw, mask_is_left, mask_is_right, car_pos, car_dir)

            generated_path = Path()
            generated_path.header.frame_id = "odom"
            generated_path.header.stamp = self.stamp
            for pose in path:

                poseStamp = PoseStamped()
                poseStamp.header.frame_id = "odom"
                poseStamp.header.stamp = self.stamp
                poseStamp.pose.position.x = pose[1]
                poseStamp.pose.position.y = pose[2]
                generated_path.poses.append(poseStamp)



        # running path planner with false cones

        if not both_cones:
            cones_right_raw = np.array([])
            mask_is_left = np.ones(len(cones_left_raw), dtype=bool)
            cones_left_adjusted = cones_left_raw - car_pos
            mask_is_left[np.argsort(np.linalg.norm(cones_left_adjusted, axis=1))[5:]] = False
            mask_is_right = np.zeros(len(cones_right_raw), dtype=bool)

        false_cones = np.array([(marker.pose.position.x, marker.pose.position.y) for marker in self.false_cones.markers])

        path = self.run_path_planner(cones_left_raw, cones_right_raw, mask_is_left, mask_is_right, car_pos, car_dir, false_cones, uncolored=False, both_cones=both_cones)
        
        uncolored_path = Path()
        uncolored_path.header.frame_id = "odom"
        uncolored_path.header.stamp = self.stamp
        for pose in path:
            poseStamp = PoseStamped()
            poseStamp.header.frame_id = "odom"
            poseStamp.header.stamp = self.stamp
            poseStamp.pose.position.x = pose[1]
            poseStamp.pose.position.y = pose[2]
            uncolored_path.poses.append(poseStamp)

        self.new_path = generated_path

        self.uncolored_path_pub.publish(uncolored_path)
        self.generated_path_pub.publish(generated_path)
        
        

    def run_path_planner(self, cones_left_raw, cones_right_raw, mask_left, mask_right, car_pos, car_dir, false_cones= np.array([]), uncolored = False, both_cones = True):
        """
        Runs the path planner with the given cones and car position.
        Args:
        - cones_left_raw (n x 2)
        - cones_right_raw (n x 2)
        - mask_left (n x 1)
        - mask_right (n x 1)
        - car_pos ([x,y])
        - car_dir ([x,y])
        - false_cones (n x 2)
        - uncolored (bool) : If True, the path planner will not differentiate between left and right cones.
        - both_cones (bool)
        Return:
        - path (n x 2) : The generated path in the odometry frame.
        """
        
        if not uncolored:
            cones_left = cones_left_raw[mask_left]
            cones_right = cones_right_raw[mask_right] if both_cones else np.array([])
            
            if false_cones.size == 0:
                cones_unknown = np.row_stack(
                    [cones_left_raw[~mask_left], cones_right_raw[~mask_right]]
                ) if both_cones else cones_left_raw[~mask_left]
            else:
                cones_unknown = np.row_stack(
                    [cones_left_raw[~mask_left], cones_right_raw[~mask_right], false_cones]) if both_cones else np.row_stack([cones_left_raw[~mask_left], false_cones])

        else : 
            cones_left = np.array([])
            cones_right = np.array([])
            if false_cones.size == 0:
                cones_unknown = np.row_stack(
                    [cones_left_raw, cones_right_raw]
                ) if both_cones else cones_left_raw
            else:
                cones_unknown = np.row_stack(
                [cones_left_raw, cones_right_raw, false_cones]
            ) if both_cones else np.row_stack([cones_left_raw, false_cones])
        


        cones_by_type = [np.zeros((0, 2)) for _ in range(5)]
        cones_by_type[ConeTypes.LEFT] = cones_left
        cones_by_type[ConeTypes.RIGHT] = cones_right
        cones_by_type[ConeTypes.UNKNOWN] = cones_unknown

        out = planner.calculate_path_in_global_frame(
            cones_by_type, car_pos, car_dir, return_intermediate_results=True
        )

        (
            path,
            sorted_left,
            sorted_right,
            left_cones_with_virtual,
            right_cones_with_virtual,
            left_to_right_match,
            right_to_left_match,
        ) = out

        return path

    def customPath_to_path(self, event):
        """
        Converts the custom path to a Path message and publishes it.
        
        """
        converted_path = Path()
        if self.curr_path is not None and self.markers is not None:
            converted_path.header.frame_id = "odom"
            converted_path.header.stamp = self.stamp
            for x,y in zip(self.curr_path.x, self.curr_path.y):
                poseStamp = PoseStamped()
                poseStamp.header.frame_id = "odom"
                poseStamp.header.stamp = self.stamp
                poseStamp.pose.position.x = x
                poseStamp.pose.position.y = y
                converted_path.poses.append(poseStamp)
            self.original_path_pub.publish(converted_path)
            
            self.create_false_cones(self.markers.markers)
            



    def curr_path_callback(self, curr_path):
        self.curr_path = curr_path
        # rospy.loginfo(self.curr_path)

    def marker_callback(self, markers):
        global seen_right_cones, seen_left_cones
        """Callback function for the marker subscriber. Updates the detected cones and generate false cones"""
        self.markers = markers
        self.cones_right_raw = np.array([])
        self.cones_left_raw = np.array([])
        for i,marker in enumerate(self.markers.markers):
            position = np.array([marker.pose.position.x, marker.pose.position.y])
            if marker.color.b == 1.0:
                if self.cones_right_raw.size == 0:  # Initialize if empty
                    self.cones_right_raw = position
                else:
                    self.cones_right_raw = np.vstack((self.cones_right_raw, position))
                    
            else:
                if self.cones_left_raw.size == 0:  # Initialize if empty
                    self.cones_left_raw = position
                else:
                    self.cones_left_raw = np.vstack((self.cones_left_raw, position))
        

        
            
            
        
    def filter_markers(self, left_cones, right_cones):
        left_cones = np.unique(left_cones, axis=0)
        right_cones = np.unique(right_cones, axis=0)
        combined_cones = np.vstack((left_cones, right_cones))
        #find duplicate cones from combined_cones
        # Convert to pandas DataFrame for easier manipulation
        df = pd.DataFrame(combined_cones, columns=['x', 'y'])
        
        # Find duplicates
        duplicate_cones = df[df.duplicated()]
        
        if duplicate_cones.size != 0:
            print("Duplicate cones found")
            # Remove duplicates from cones_left_raw
            cones_left_df = pd.DataFrame(left_cones, columns=['x', 'y'])
            left_cones = pd.concat([duplicate_cones, cones_left_df]).drop_duplicates(keep=False).to_numpy()
            return left_cones,right_cones
        return left_cones, right_cones

    def create_false_cones(self, cones, generate_radius=4):
        r = cones[0].scale.x
        if random.random() < 0.1:
            # Generate a random radius between 1.2r and 1.5r
            random_radius = random.uniform(1.4 * r, 2.0 * r)

            # Generate a random angle between 0 and 2π
            random_angle = random.uniform(0, 2 * math.pi)
            potential_cones = []
            #iterate through the cones, find the cones inside the car radiusx   
            for cone in cones:
                cone_pos = np.array([cone.pose.position.x, cone.pose.position.y])
                car_pos = np.array([self.car_x, self.car_y])
                if math.dist(cone_pos, car_pos) < generate_radius:
                    potential_cones.append(cone)

                    
            if potential_cones != []:
                new_cone = deepcopy(random.choice(potential_cones))
                # Convert polar coordinates to Cartesian coordinates
                new_x = cone.pose.position.x + random_radius * math.cos(random_angle)
                new_y = cone.pose.position.y + random_radius * math.sin(random_angle)   
                new_cone.pose.position.x = new_x
                new_cone.pose.position.y = new_y
                new_cone.color.a = 1.0
                new_cone.color.r = 1.0
                new_cone.color.g = 1.0
                new_cone.color.b = 1.0
                self.false_cones.markers.append(new_cone)      

    def odom_callback(self, odom):
        """
        Callback function for the odometry subscriber. Updates the car's position and orientation.
        """
        self.stamp = odom.header.stamp
        self.car_x = odom.pose.pose.position.x
        self.car_y = odom.pose.pose.position.y
        quart_angle = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        self.car_yaw = euler_from_quaternion(quart_angle)[2]
        self.generate_marker(odom)
        self.car_position = np.array([self.car_x, self.car_y])
        self.car_direction = unit_2d_vector_from_angle(self.car_yaw)
        
        
    def generate_marker(self, odom):
        """
        Generates and publish a marker for car heading.
        """
        heading = Marker()
        heading.header.frame_id = "odom"
        heading.header.stamp = odom.header.stamp
        heading.type = 0
        heading.id = 100
        heading.scale.x = 0.7
        heading.scale.y = 0.15
        heading.scale.z = 0.15
        heading.pose.position.x = self.car_x
        heading.pose.position.y = self.car_y
        heading.pose.position.z = 0
        heading.pose.orientation.x = odom.pose.pose.orientation.x
        heading.pose.orientation.y = odom.pose.pose.orientation.y
        heading.pose.orientation.z = odom.pose.pose.orientation.z
        heading.pose.orientation.w = odom.pose.pose.orientation.w
        heading.color.a = 1.0
        heading.color.r = 1.0

        self.generate_heading_pub.publish(heading)

if __name__ == "__main__":    
    try:
        node = PlannerNode()
        node.run_node()
    except rospy.ROSInterruptException:
        pass