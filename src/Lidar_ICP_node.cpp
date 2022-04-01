#include <Lidar_ICP/Lidar_ICP.h>

int main (int argc, char *argv[])
{   
    ros::init(argc, argv, "Lidar_ICP_2d");
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;
    Lidar_ICP *lidar_ICP = new Lidar_ICP;
    ros::Rate rate(4);
    while (ros::ok())
    {
        lidar_ICP->ICP();
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "laser1" ;
        transformStamped.child_frame_id = "laser2";
        transformStamped.transform.translation.x = lidar_ICP->get_transx();
        transformStamped.transform.translation.y = lidar_ICP->get_transy();
        transformStamped.transform.translation.z = 0.0;
        q.setRPY(0, 0, lidar_ICP->get_rotation());
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);

        ros::spinOnce();   
        rate.sleep();
    }
    return 0;
}

            
            
            
            
            
