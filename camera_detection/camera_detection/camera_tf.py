#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import transforms3d._gohlketransforms as t3  # for quaternion math
import tf2_ros
from geometry_msgs.msg import TransformStamped

class CameraPoseInKinova(Node):
    def __init__(self):
        super().__init__('camera_pose_in_kinova')

        # 1) Create TF2 buffer & listener to read all transforms:
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 2) Create a static broadcaster so we can publish kinova->tag0 once:
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.kinova_base_link_frame = 'kinova/base_link'
        self.tag0_anchor_frame = 'tag0_anchor'
        self.camera_frame = 'table_camera/camera_link/rgb_camera'
        self.tag_frame = 'tag36h11:0'

        # 3) Define the known “kinova/base_link → tag0_anchor” transform:
        #    (x=0.10, y=0.00, z=0.75), yaw=0, pitch=0, roll=0
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id    = self.kinova_base_link_frame
        static_tf.child_frame_id     = self.tag0_anchor_frame
        static_tf.transform.translation.x = 0.10
        static_tf.transform.translation.y = 0.00
        static_tf.transform.translation.z = 0.75
        # rotation quaternion for roll=0,pitch=0,yaw=0
        q = t3.quaternion_from_euler(0.0, 0.0, 0.0)
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]

        # publish it once on /tf_static
        self.static_broadcaster.sendTransform(static_tf)
        self.get_logger().info(
            "Published static kinova/base_link → tag0_anchor"
        )

        # 4) Create a dynamic broadcaster to publish kinova_to_camera
        self.dyn_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 5) Timer to recompute & broadcast “kinova/base_link → camera_frame”
        timer_period = 1.0 / 10.0  # 10 Hz
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.warned_about_missing_transform = False

    def on_timer(self):
        # Attempt to look up the dynamic transform: camera_frame → tag36h11:0
        # Replace "camera_frame" with whatever your actual camera frame is

        try:
            # We use latest available (time=0.0) so it picks the last‐seen transform:
            cam_to_tag = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.tag_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            if not self.warned_about_missing_transform:
                self.get_logger().warn(
                    f"Could not look up {self.camera_frame} -> {self.tag_frame}: {e}"
                )
                self.warned_about_missing_transform = True
            return

        # Invert it to get tag -> camera
        t = cam_to_tag.transform.translation
        r = cam_to_tag.transform.rotation
        # Build a 4×4 matrix for camera->tag
        mat_ct = t3.concatenate_matrices(
            t3.translation_matrix((t.x, t.y, t.z)),
            t3.quaternion_matrix((r.x, r.y, r.z, r.w))
        )
        # Invert to get tag->camera
        mat_tc = t3.inverse_matrix(mat_ct)

        # Now look up the static kinova/base_link -> tag0_anchor
        try:
            kinova_to_tag_static = self.tf_buffer.lookup_transform(
                self.kinova_base_link_frame,
                self.tag0_anchor_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not look up static kinova/base_link -> tag0_anchor: {e}"
            )
            return

        # Construct 4×4 matrix for kinova->tag (static)
        ts = kinova_to_tag_static.transform.translation
        rs = kinova_to_tag_static.transform.rotation
        mat_kt = t3.concatenate_matrices(
            t3.translation_matrix((ts.x, ts.y, ts.z)),
            t3.quaternion_matrix((rs.x, rs.y, rs.z, rs.w))
        )

        # Now chain: kinova->tag -> (tag->camera) = kinova->camera
        mat_kc = t3.concatenate_matrices(mat_kt, mat_tc)

        # Extract translation and rotation quaternion for kinova->camera
        trans = t3.translation_from_matrix(mat_kc)
        quat  = t3.quaternion_from_matrix(mat_kc)

        # Publish it as a dynamic TF
        dyn_tf = TransformStamped()
        dyn_tf.header.stamp = cam_to_tag.header.stamp
        dyn_tf.header.frame_id    = self.kinova_base_link_frame
        dyn_tf.child_frame_id     = self.camera_frame
        dyn_tf.transform.translation.x = trans[0]
        dyn_tf.transform.translation.y = trans[1]
        dyn_tf.transform.translation.z = trans[2]
        dyn_tf.transform.rotation.x = quat[0]
        dyn_tf.transform.rotation.y = quat[1]
        dyn_tf.transform.rotation.z = quat[2]
        dyn_tf.transform.rotation.w = quat[3]

        self.dyn_broadcaster.sendTransform(dyn_tf)
        self.get_logger().info(
            f"Published {self.kinova_base_link_frame} -> {self.camera_frame}: t=[{trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f}]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseInKinova()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
