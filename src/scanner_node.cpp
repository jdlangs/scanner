#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

const int NUM_CLOUDS = 5;

bool loadPointClouds(std::string dir, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &point_clouds)
{
  for (int i=0; i < NUM_CLOUDS; ++i)
  {
    std::string file = dir + "/scan" + std::to_string(i+1) + ".pcd";
    ROS_INFO("Loading point cloud '%s'", file.c_str());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1)
    {
      ROS_ERROR("Couldn't read file %s", file.c_str());
      return false;
    }
    point_clouds.push_back(std::move(cloud));
  }
  return true;
}

class ScannerNode
{
public:
  ScannerNode(ros::NodeHandle &nh, ros::NodeHandle &nhp) : current_index_(0)
  {
    std::string dir;
    if (nhp.getParam("scan_directory", dir))
    {
      loadPointClouds(dir, point_clouds_);
    }
    else
    {
      ROS_ERROR("Required 'scan_directory' parameter not set");
    }

    //TODO: advertise a 'get_scan' service and save the result in the `scan_service_` object
    //TODO: advertise a 'cloud' topic and save the result in the `cloud_publisher_` object
  }

  bool getScanCallback(/*TODO: the service response and request types*/)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan = getCurrentScan();
    //
    //TODO:
    //  Step 1: convert pcl cloud type to ros sensor_msgs type
    //     Note: you need to specify the coordinate frame inside the `header.frame_id` of the message. Use the frame of
    //     the conveyor.
    //  Step 2: Put the cloud into the service response
    //  Step 3: Publish the cloud using the publisher created for inspecting the scan in rviz

    return true;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr getCurrentScan()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = point_clouds_.at(current_index_);
    current_index_++;
    if (current_index_ >= point_clouds_.size())
      current_index_ = 0;
    return cloud;
  }

private:
  int current_index_;

  ros::ServiceServer scan_service_;
  ros::Publisher cloud_publisher_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds_;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "scanner_node");
  ros::NodeHandle nh, nhp("~");

  ScannerNode node(nh, nhp);

  ros::spin();
  return 0;
}
