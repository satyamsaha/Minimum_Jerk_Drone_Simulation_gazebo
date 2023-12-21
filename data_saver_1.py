import rospy
import csv
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State, RCOut
from message_filters import ApproximateTimeSynchronizer, Subscriber
import message_filters
from tf.transformations import euler_from_quaternion

# Initialize synchronized data lists
synchronized_data = []
d=0.18
cT=0.00051566
cM=1.3785541*(10**(-5))

def synchronized_callback(velocity_msg, pose_msg, rcout_msg):
    timestamp = rospy.Time.now().to_sec()

    velocity_data = [velocity_msg.twist.linear.x, velocity_msg.twist.linear.y, velocity_msg.twist.linear.z]
    angular_velocity_data = [velocity_msg.twist.angular.x, velocity_msg.twist.angular.y, velocity_msg.twist.angular.z]
    pose_orientation = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]
    pose_data = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]
    pwm_values = list(rcout_msg.channels[:4]) 
    thrust_values=[3.61588461*(10**(-4))*pwm_values[0]-0.378894041,3.61588461*(10**(-4))*pwm_values[1]-0.378894041,3.61588461*(10**(-4))*pwm_values[2]-0.378894041,3.61588461*(10**(-4))*pwm_values[3]-0.378894041]
    total_thrust=((3.61588461*(10**(-4))*pwm_values[0]-0.378894041)+(3.61588461*(10**(-4))*pwm_values[1]-0.378894041)+(3.61588461*(10**(-4))*pwm_values[2]-0.378894041)+(3.61588461*(10**(-4))*pwm_values[3]-0.378894041))
    rpm1=-0.0135731379*(pwm_values[0]**2)+42.0025634*pwm_values[0]-28306.0077
    rpm2=-0.0135731379*(pwm_values[1]**2)+42.0025634*pwm_values[1]-28306.0077
    rpm3=-0.0135731379*(pwm_values[2]**2)+42.0025634*pwm_values[2]-28306.0077
    rpm4=-0.0135731379*(pwm_values[3]**2)+42.0025634*pwm_values[3]-28306.0077
    rpm_values=[rpm1,rpm2,rpm3,rpm4]
    moment_x= d*cT*(0.707*(rpm1**2)-0.707*(rpm2**2)-0.707*(rpm3**2)+0.707*(rpm2**2))*((2*np.pi/60)**2)
    moment_y= d*cT*(0.707*(rpm1**2)+0.707*(rpm2**2)-0.707*(rpm3**2)-0.707*(rpm2**2))*((2*np.pi/60)**2)
    moment_z= cM*((rpm1**2)-(rpm2**2)+(rpm3*2)-(rpm4**2))*((2*np.pi/60)**2)
    moments=[moment_x,moment_y,moment_z]
    (roll, pitch, yaw) = euler_from_quaternion (pose_orientation)
    synchronized_data.append([timestamp] + velocity_data + angular_velocity_data + pose_data+[roll]+[pitch]+[yaw] + pwm_values+thrust_values+[total_thrust]+rpm_values+moments)
    
if __name__ == "__main__":
    rospy.init_node("data_recorder")

    velocity_sub = message_filters.Subscriber("/mavros/local_position/velocity_local", TwistStamped) #
    pose_sub = message_filters.Subscriber("/mavros/local_position/pose", PoseStamped)
    rcout_sub = message_filters.Subscriber("/mavros/rc/out", RCOut)

    ts = ApproximateTimeSynchronizer([velocity_sub, pose_sub, rcout_sub], queue_size=100, slop=0.1)
    ts.registerCallback(synchronized_callback)

    rate = rospy.Rate(100)  # Recording rate in Hz

    while not rospy.is_shutdown():
        rate.sleep()

        if len(synchronized_data) > 0:
            # Print some rows from synchronized_data for debugging
            print("Sample synchronized_data row:", synchronized_data[-1])

        # Get the current state of the drone
        current_state = None  # Initialize with None
        try:
            current_state = rospy.wait_for_message("/mavros/state", State, timeout=1.0)
        except rospy.ROSException:
            pass

        # Check if the drone is armed and guided to start recording
        recording_started = current_state and current_state.armed and current_state.guided

        if recording_started:
            rospy.loginfo("Recording started.")
            break  # Exit loop after recording starts

    rospy.sleep(1.0)  # Add a delay to ensure enough data is collected

    while not rospy.is_shutdown():
        rate.sleep()

        if recording_started:
            # Save synchronized data to CSV file
            file_name = "synchronized_data3.csv"
            with open(file_name, "w") as csv_file:
                writer = csv.writer(csv_file)
                header_row = ["Time",
                              "Linear_Vel_X", "Linear_Vel_Y", "Linear_Vel_Z",
                              "Angular_Vel_X", "Angular_Vel_Y", "Angular_Vel_Z",
                              "Position_X", "Position_Y", "Position_Z",
                              "roll", "pitch", "yaw",
                              "PWM1", "PWM2", "PWM3", "PWM4","Thrust1","Thrust2","Thrust3","Thrust4","total_thrust","rpm1","rpm2","rpm3","rpm4","moment_x","moment_y","moment_z"]
                writer.writerow(header_row)

                for row in synchronized_data:
                    writer.writerow(row)

            rospy.loginfo("Data saved to {}".format(file_name))
            break  # Exit loop after recording and saving data

    rospy.loginfo("Data collection complete.")
