#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

class PointCloudClipper {
public:
    PointCloudClipper()
    {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("~");

        // Get parameters from the parameter server
        nh_.param("min_z", min_z_, -0.8);
        nh_.param("max_z", max_z_, 5.0);

        // Subscribe to input point cloud topic
        sub_ = nh_.subscribe("/velodyne_points", 1, &PointCloudClipper::pointCloudCallback, this);

        // Advertise output point cloud topic
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // Convert the sensor_msgs/PointCloud2 message to pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input, *cloud);

        // Find the minimum z value in the input point cloud
        double min_z_in_cloud = std::numeric_limits<double>::max();
        for (const auto& point : cloud->points) {
            if (point.z < min_z_in_cloud) {
                min_z_in_cloud = point.z;
            }
        }

        // Create the passthrough filter to clip the point cloud
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_z_, max_z_);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pass.filter(*cloud_filtered);

        // Convert the filtered pcl::PointCloud back to sensor_msgs/PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);

        // Set the frame ID and timestamp
        output.header.frame_id = input->header.frame_id;
        output.header.stamp = input->header.stamp;

        // Publish the filtered point cloud
        pub_.publish(output);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double min_z_;
    double max_z_;
};

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "point_cloud_clipper");

    // Create the PointCloudClipper object
    PointCloudClipper clipper;

    // Spin to process incoming messages
    ros::spin();

    return 0;
}

