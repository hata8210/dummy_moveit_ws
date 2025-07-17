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
        super().__init__('aruco_detector')
        self.sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.br = CvBridge()
        self.pub = self.create_publisher(PoseStamped, '/aruco_target_pose', 10)
        self.pub_cam = self.create_publisher(PoseStamped, '/aruco_camera_pose', 10)
        self.pub_world = self.create_publisher(PoseStamped, '/aruco_world_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # camera_color_optical_frame from ros2 camera_info topic, k is camera_matrix, d is dist_coeffs 
        self.camera_matrix = np.array([
            [914.0535, 0.0, 647.0988],
            [0.0, 914.5477, 372.1199],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([0, 0, 0, 0, 0])
        self.marker_length = 0.05  # meter

        # publish pose in every 1 seconds
        self.last_pose_tool = None
        self.last_pose_cam = None
        self.last_pose_world = None
        self.create_timer(1.0, self.publish_cached_poses)

    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)  #DICT_ARUCO_ORIGINAL ,DICT_4X4_50
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
        if ids is not None:
            #self.get_logger().info('okok,detect aruco size:',len(ids))
            # use cv2 function to tran image pose to space pose
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            # use custom function to tran image pose to space pose
            #rvec_m, tvec_m, _ = self.my_estimate_pose_single_markers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            # just take the first marker
            pos_x = tvec[0][0][0]
            pos_y = tvec[0][0][1]
            pos_z = tvec[0][0][2]
            orien_x = rvec[0][0][0]
            orien_y = rvec[0][0][1]
            orien_z = rvec[0][0][2]
            # compare result between two function
            #self.get_logger().info('check,my_estimate_pose_single_markers.pos_x:'+tvec[0][0][0]+'=='+tvec_m[0][0][0])
            #self.get_logger().info('check,my_estimate_pose_single_markers.pos_y:'+tvec[0][0][0]+'=='+tvec_m[0][0][0])
            #self.get_logger().info('check,my_estimate_pose_single_markers.pos_z:'+tvec[0][0][0]+'=='+tvec_m[0][0][0])
            #self.get_logger().info('check,my_estimate_pose_single_markers.orien_x:'+tvec[0][0][0]+'=='+tvec_m[0][0][0])
            #self.get_logger().info('check,my_estimate_pose_single_markers.orien_y:'+tvec[0][0][0]+'=='+tvec_m[0][0][0])
            #self.get_logger().info('check,my_estimate_pose_single_markers.orien_z:'+tvec[0][0][0]+'=='+tvec_m[0][0][0])
            
            # set camera_color_optical_frame pose
            pose_in_optical = PoseStamped()
            pose_in_optical.header.stamp = self.get_clock().now().to_msg()
            pose_in_optical.header.frame_id = 'camera_color_optical_frame'
            pose_in_optical.pose.position.x = pos_x
            pose_in_optical.pose.position.y = pos_y
            pose_in_optical.pose.position.z = pos_z
            quat = tf_transformations.quaternion_from_euler(orien_x,orien_y,orien_z)
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
            self.get_logger().info('okok,now pose_in_tool frame_id:'+pose_in_tool.header.frame_id+', pose_in_world frame_id:'+pose_in_world.header.frame_id)

            # save pose in cache
            #self.pub.publish(pose_in_tool)
            #self.pub_cam.publish(pose_in_optical)
            #self.pub_world.publish(pose_in_world)
            self.last_pose_tool = pose_in_tool
            self.last_pose_cam = pose_in_optical
            self.last_pose_world = pose_in_world
        else:
            self.get_logger().info('oh,can not detect any aruco')
    
    def my_estimate_pose_single_markers(self,corners, marker_length, camera_matrix, dist_coeffs):
        """
        use solvePnP to replace cv2.aruco.estimatePoseSingleMarkers
        """
        # ArUco origin shape (use meter)
        half_len = marker_length / 2.0
        objp = np.array([
            [-half_len,  half_len, 0],
            [ half_len,  half_len, 0],
            [ half_len, -half_len, 0],
            [-half_len, -half_len, 0]
        ], dtype=np.float32)
        rvecs = []
        tvecs = []
        for corner in corners:
            # corner take (4, 2) from (1, 4, 2)
            imgp = corner[0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs)
            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)
            else:
                rvecs.append(None)
                tvecs.append(None)
        # tran to numpy, the same with estimatePoseSingleMarkers
        return np.array(rvecs), np.array(tvecs), None

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
