// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//local lib
#include "lidar.h"
#include "laserProcessingClass.h"


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

std::string lidar_type;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;

size_t convertMyLidarToPCLPointCloud(const sensor_msgs::PointCloud2ConstPtr &pcd_msg, pcl::PointCloud<pcl::PointXYZI> &pcd)
{
  if (pcd_msg)
  {
    size_t pt_size = pcd_msg->width * pcd_msg->height;

    // output.points.resize (pcd_msg->width * pcd_msg->height);
    // output.channels.resize (pcd_msg->fields.size () - 3);
    // Get the x/y/z field offsets
    int x_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "x");
    int y_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "y");
    int z_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "z");
    int inten_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "intensity");
    
    int x_offset = pcd_msg->fields[x_idx].offset;
    int y_offset = pcd_msg->fields[y_idx].offset;
    int z_offset = pcd_msg->fields[z_idx].offset;
    int inten_offset = pcd_msg->fields[inten_idx].offset;

    uint8_t x_datatype = pcd_msg->fields[x_idx].datatype;
    uint8_t y_datatype = pcd_msg->fields[y_idx].datatype;
    uint8_t z_datatype = pcd_msg->fields[z_idx].datatype;
    uint8_t inten_datatype = pcd_msg->fields[inten_idx].datatype;

    // Copy the data points
    size_t validCount = 0;
    for (size_t cp = 0; cp < pt_size; ++cp)
    {
      // Copy x/y/z/intensity
      pcl::PointXYZI pt;
      pt.x = sensor_msgs::readPointCloud2BufferValue<float>(&pcd_msg->data[cp * pcd_msg->point_step + x_offset], x_datatype);
      pt.y = sensor_msgs::readPointCloud2BufferValue<float>(&pcd_msg->data[cp * pcd_msg->point_step + y_offset], y_datatype);
      pt.z = sensor_msgs::readPointCloud2BufferValue<float>(&pcd_msg->data[cp * pcd_msg->point_step + z_offset], z_datatype);

      if (inten_idx >= 0)
        pt.intensity = sensor_msgs::readPointCloud2BufferValue<uint8_t>(&pcd_msg->data[cp * pcd_msg->point_step + inten_offset], inten_datatype);

      pcd.push_back(pt);
      validCount++;
    }
    return validCount;
  }
  return 0;
}

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
   
}

double total_time =0;
int total_frame=0;

void laser_processing(){
    while(1){
        if(!pointCloudBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            if(lidar_type == "rsbpearl")
                convertMyLidarToPCLPointCloud(pointCloudBuf.front(), *pointcloud_in);
            else
               pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            if(lidar_type == "rsbpearl"){
                laserProcessing.rsbp_featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            }else
                laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "/base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            // sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            // pcl::toROSMsg(*pointcloud_in, laserCloudFilteredMsg);
            // laserCloudFilteredMsg.header.stamp = pointcloud_time;
            // laserCloudFilteredMsg.header.frame_id = "/base_link";
            // pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "/base_link";
            pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "/base_link";
            pubSurfPoints.publish(surfPointsMsg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("lidar_type",lidar_type);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler);

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1000);

    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 10000);

    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 10000); 

    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
}

