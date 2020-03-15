#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

main (int argc, char **argv)
{
    ros::init (argc, argv, "laser_rviz_pt");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("laser_pt", 1);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    sensor_msgs::PointCloud2 output;

    // Fill in the cloud data
    cloud.width  = 50000;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].r = 0;
        cloud.points[i].g = 255;
        cloud.points[i].b = 0;
        cloud.points[i].x = 0;
        cloud.points[i].y = 2 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 2 * rand () / (RAND_MAX + 1.0f);
    }

    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "laser_plane";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}