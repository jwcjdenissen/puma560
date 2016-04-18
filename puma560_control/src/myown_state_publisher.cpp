#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "myown_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pubisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;

    ros::Rate loop_rate(30);

    // robot state
    double angle=0.0;

    // message declarations
    sensor_msgs::JointState joint_state;
    geometry_msgs::TransformStamped tf_frame;
    tf_frame.header.frame_id = "world";
    tf_frame.child_frame_id = "puma_link_5";


    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(5);
        joint_state.position.resize(5);
        joint_state.name[0] ="joint_1";
        joint_state.position[0] = angle;
        joint_state.name[1] ="joint_2";
        joint_state.position[1] = angle;
        joint_state.name[2] ="joint_3";
        joint_state.position[2] = angle;
        joint_state.name[3] ="joint_4";
        joint_state.position[3] = 0;
        joint_state.name[4] ="joint_5";
        joint_state.position[4] = 0;

        // update transform
        // (moving in a circle with radius=2)
        tf_frame.header.stamp = ros::Time::now();
        tf_frame.transform.translation.x = cos(angle)*2;
        tf_frame.transform.translation.y = sin(angle)*2;
        tf_frame.transform.translation.z = .7;

        //send the joint state and transform
        joint_pubisher.publish(joint_state);

        // Create new robot state
        angle += 0.01;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}