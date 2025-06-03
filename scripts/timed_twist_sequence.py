# save as timed_twist_sequence.py
import rclpy, time
from geometry_msgs.msg import TwistStamped

# ---------- 1.  Define your sequence (linear.x, angular.z, duration) ----------
SEQ = [
    (2.0,  4.0, 5.6),   # forward 2 m s⁻¹ + yaw  +2.0 rad s⁻¹
    (2.2,  0.0, 27.0),   # forward 2 m s⁻¹ + yaw  –0.5 rad s⁻¹
    (2.0,  -2.5, 1.6),   # forward 2 m s⁻¹ + yaw  +0.5 rad s⁻¹
]
HZ = 10.0               # publishing rate (messages per second)
# ------------------------------------------------------------------------------

def main():
    rclpy.init()
    node = rclpy.create_node('timed_twist_sequence')
    pub  = node.create_publisher(TwistStamped, '/j100_0000/cmd_vel', 1)

    msg = TwistStamped()
    msg.header.frame_id = 'base_link'
    period = 1.0 / HZ

    for lin_x, ang_z, secs in SEQ:
        msg.twist.linear.x  = lin_x
        msg.twist.angular.z = ang_z
        t_end = time.time() + secs
        while time.time() < t_end:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            time.sleep(period)

    # ensure the robot stops afterwards
    msg.twist.linear.x = msg.twist.angular.z = 0.0
    for _ in range(int(HZ*0.5)):               # 0.5 s of zero cmd to flush
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
