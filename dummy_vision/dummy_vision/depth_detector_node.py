# simple Python demo using package cv2 + cv_bridge
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import tf_transformations
import tf2_ros
import numpy as np

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from tf_transformations import quaternion_multiply, quaternion_matrix



class ArucoDetector(Node):
    def __init__(self):
        super().__init__('depth_detector')
        
        # detect status
        self.sending = False   # True means pose is ready and publish next time
        self.calculating = False  # True means target have been discovered and calculate target pose
        self.target_img_pose = [0,0]   # [x,y] pixel of target in image
        self.target_depth = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.orien_x = 0.0
        self.orien_y = 0.0
        self.orien_z = 0.0
        self.temp_x = 0.0
        self.temp_y = 0.0
        self.temp_z = 0.0
        
        # subscribe topics
        self.sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        # self.sub_depth = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        # camera setting
        self.br = CvBridge()
        self.pub = self.create_publisher(PoseStamped, '/aruco_target_pose', 10)
        self.pub_cam = self.create_publisher(PoseStamped, '/aruco_camera_pose', 10)
        self.pub_world = self.create_publisher(PoseStamped, '/aruco_world_pose', 10)
        # camera_color_optical_frame from ros2 camera_info topic, k is camera_matrix, d is dist_coeffs 
        self.camera_matrix = np.array([
            [914.0535, 0.0, 647.0988],
            [0.0, 914.5477, 372.1199],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([0, 0, 0, 0, 0])
        
        # ros2 tf setting
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # target setting
        self.marker_length = 0.05  # meter

        # publish pose in every 1 seconds
        self.last_pose_tool = None
        self.last_pose_cam = None
        self.last_pose_world = None
        self.create_timer(1.0, self.publish_cached_poses)

    def depth_callback(self, msg):
        """
        calculating the depth by back projection
        """
        if self.calculating == False :
            return None
        else:
            #self.get_logger().info('oh,calculating the depth distance ,self.target_img_pose:'+str(self.target_img_pose))
            
            # target pose to depth image
            x, y = int(self.target_img_pose[0]), int(self.target_img_pose[1])
            # color image pixel align depth image pixel
            #.......color:1280x720,depth:848,480
            # transform to OpenCV format
            depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            #self.get_logger().info(f"depth_image x,y size: "+str(depth_image.shape[1])+","+str(depth_image.shape[0]))
            if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                #self.get_logger().info(f"target_depth: "+str(depth_image[y, x]))
                # unit 16UC1 is milimeter ,unit 32FC1 is meter ,16UC1 for default , it s meter here
                self.target_depth = depth_image[y, x]/1000 
                self.get_logger().info(f"depth of pose({x}, {y}): {self.target_depth}")
            else:
                self.get_logger().warn("exception:out of range in depth image!!")

            # calculate depth
            coords_temp = self.pixel_to_camera_coords(x, y, self.target_depth, self.camera_matrix)

            # set target coords in camera frame
            self.pos_x = coords_temp[0]
            self.pos_y = coords_temp[1]
            self.pos_z = coords_temp[2]
            self.orien_x = 0.0
            self.orien_y = 0.0
            self.orien_z = 0.0
            self.calculating = False
            self.get_logger().info('check,pixel_to_camera_coords.pos_x:'+str(round(coords_temp[0],2))+' vs '+str(round(self.temp_x,2)))
            self.get_logger().info('check,pixel_to_camera_coords.pos_y:'+str(round(coords_temp[1],2))+' vs '+str(round(self.temp_y,2)))
            self.get_logger().info('check,pixel_to_camera_coords.pos_z:'+str(round(coords_temp[2],2))+' vs '+str(round(self.temp_z,2)))

            # set camera_color_optical_frame pose
            pose_in_optical = PoseStamped()
            pose_in_optical.header.stamp = self.get_clock().now().to_msg()
            pose_in_optical.header.frame_id = 'camera_color_optical_frame'
            #self.get_logger().info('check,pos_x value:'+str(self.pos_x))
            #self.get_logger().info('check,pos_x type:'+str(type(self.pos_x)))
            pose_in_optical.pose.position.x = self.pos_x
            pose_in_optical.pose.position.y = self.pos_y
            pose_in_optical.pose.position.z = self.pos_z
            quat = tf_transformations.quaternion_from_euler(self.orien_x,self.orien_y,self.orien_z)
            pose_in_optical.pose.orientation.x = quat[0]
            pose_in_optical.pose.orientation.y = quat[1]
            pose_in_optical.pose.orientation.z = quat[2]
            pose_in_optical.pose.orientation.w = quat[3]
            
            # make camera_color_optical_frame turn to camera_link, or straight to link6_1_1
            #self.get_logger().info('it frames in buffer:')
            #self.get_logger().info(self.tf_buffer.all_frames_as_string())
            try:
                self.tf_buffer.can_transform("link6_1_1", "camera_color_optical_frame", rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0))
                transform = self.tf_buffer.lookup_transform("link6_1_1", "camera_color_optical_frame", rclpy.time.Time())
                self.tf_buffer.can_transform("base_link", "camera_color_optical_frame", rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0))
                transform_world = self.tf_buffer.lookup_transform("base_link", "camera_color_optical_frame", rclpy.time.Time())
                #self.get_logger().warn("TF from camera_color_optical_frame to link6_1_1 and base_link is available!!!")
            except:
                self.get_logger().warn("TF from camera_color_optical_frame to link6_1_1 or base_link not available yet")
            pose_in_tool = self.transform_pose(pose_in_optical, transform)
            pose_in_world = self.transform_pose(pose_in_optical, transform_world)
            #self.get_logger().info('okok,now pose_in_tool frame_id:'+pose_in_tool.header.frame_id+', pose_in_world frame_id:'+pose_in_world.header.frame_id)

            # save pose in cache
            #self.pub.publish(pose_in_tool)
            #self.pub_cam.publish(pose_in_optical)
            #self.pub_world.publish(pose_in_world)
            self.last_pose_tool = pose_in_tool
            self.last_pose_cam = pose_in_optical
            self.last_pose_world = pose_in_world

            self.get_logger().info('---------------------------------------')

    def image_callback(self, msg):
        # detect target in color image, for example aruco
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)  #DICT_ARUCO_ORIGINAL ,DICT_4X4_50
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
        if ids is not None and self.calculating == False:
            #self.get_logger().info('okok,detect aruco size:'+str(len(ids))+','+str(len(corners)))

            # set self.target_img_pose
            corners_target = corners[0][0]  #shape is [4,2], example [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
            #self.get_logger().info('okok,first target:'+str(corners_target))
            self.target_img_pose = self.get_center_point(corners_target)
            self.get_logger().info('okok,target_img_pose:'+str(self.target_img_pose))
            self.calculating = True


            # validate by cv method
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            self.temp_x = tvec[0][0][0]
            self.temp_y = tvec[0][0][1]
            self.temp_z = tvec[0][0][2]
            #self.get_logger().info('check,estimate_pose_single_markers.temp_x:'+str(round(tvec[0][0][0],2)))
            #self.get_logger().info('check,estimate_pose_single_markers.temp_y:'+str(round(tvec[0][0][1],2)))
            #self.get_logger().info('check,estimate_pose_single_markers.temp_z:'+str(round(tvec[0][0][2],2)))
            
            

            

    def pixel_to_camera_coords(self, u, v, depth, camera_matrix):
        """
        calculating the depth by back projection
        """
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        #return np.array([x, y, z], dtype=np.float32)
        return [x, y, z]

    def get_center_point(self, points):
        """
        find the central point by 4 corners
        :param points: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
        :return: (x_center, y_center)
        """
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        x_center = sum(x_coords) / 4.0
        y_center = sum(y_coords) / 4.0
        return [x_center, y_center]

    def publish_cached_poses(self):
        if self.last_pose_tool:
            self.pub.publish(self.last_pose_tool)
        if self.last_pose_cam:
            self.pub_cam.publish(self.last_pose_cam)
        if self.last_pose_world:
            self.pub_world.publish(self.last_pose_world)

    def transform_pose(self, pose, transform):
        # get translation and rotation from TransformStamped
        translation = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        )
        rotation = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        )

        # origin point
        point = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            1.0
        ])

        # build change matrix
        transform_mat = tf_transformations.quaternion_matrix(rotation)
        transform_mat[0:3, 3] = translation

        # take the transform
        transformed_point = np.dot(transform_mat, point)

        # get quat
        input_quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        output_quat = quaternion_multiply(rotation, input_quat)

        # build new PoseStamped
        transformed_pose = PoseStamped()
        transformed_pose.header.stamp = pose.header.stamp
        transformed_pose.header.frame_id = transform.header.frame_id
        transformed_pose.pose.position.x = transformed_point[0]
        transformed_pose.pose.position.y = transformed_point[1]
        transformed_pose.pose.position.z = transformed_point[2]
        transformed_pose.pose.orientation.x = output_quat[0]
        transformed_pose.pose.orientation.y = output_quat[1]
        transformed_pose.pose.orientation.z = output_quat[2]
        transformed_pose.pose.orientation.w = output_quat[3]

        return transformed_pose

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
