#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloud2ToPCLNodelet : public nodelet::Nodelet
{
public:
    PointCloud2ToPCLNodelet() {}
    virtual ~PointCloud2ToPCLNodelet() {}

    virtual void onInit()
    {
        nh = getNodeHandle();
        private_nh = getPrivateNodeHandle();

        // sub = nh.subscribe("input", 1, PointCloud2ToPCLNodelet::cloud_cb, this);
        // pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("output", 1, this);
    }

private:
    ros::Publisher pub;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber sub;

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        // pcl::fromROSMsg(*input, cloud);
        pcl::PCLPointCloud2 pcl_pointcloud2;
        pcl_conversions::toPCL(*input, pcl_pointcloud2);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(pcl_pointcloud2, *cloud);
        // pub.publish(cloud);
    }
};

PLUGINLIB_EXPORT_CLASS(PointCloud2ToPCLNodelet, nodelet::Nodelet)