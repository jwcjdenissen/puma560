#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "myown_tf_publisher");
    ros::NodeHandle n;
    tf::TransformBroadcaster broadcaster;

    ros::Rate loop_rate(30);

    geometry_msgs::TransformStamped tf_frame;
    tf_frame.header.frame_id = "world";
    tf_frame.child_frame_id = "puma_link_5";

    double angle=0.0;

    while (ros::ok()) {

        // update transform
        // (moving in a circle with radius=2)
        tf_frame.header.stamp = ros::Time::now();
        tf_frame.transform.translation.x = 0.5 * cos(angle)*2;
        tf_frame.transform.translation.y = 0.5 * sin(angle)*2;
        tf_frame.transform.translation.z = .7;
        tf_frame.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

        //send the joint state and transform
        broadcaster.sendTransform(tf_frame);

        // Create new robot state
        angle += 0.01;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}