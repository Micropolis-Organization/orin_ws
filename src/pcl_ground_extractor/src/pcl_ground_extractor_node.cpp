#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

class GroundPlaneSegmentation
{
public:
    GroundPlaneSegmentation() : nh("~")
    {

        // Read parameters
        nh.param<std::string>("input_topic", input_topic, "/input_point_cloud");
        nh.param<std::string>("output_topic", output_topic, "/output_point_cloud");

        // Create a ROS subscriber for the input point cloud
        sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, &GroundPlaneSegmentation::cloud_cb, this);

        // Create a ROS publisher for the output point cloud
        pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    }

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        //following this suggestion https://answers.ros.org/question/173396/losing-intensity-data-when-converting-between-sensor_msgspointcloud2-and-pclpointcloudt/ 
        //create a copy of the input point cloud
        // sensor_msgs::PointCloud2 copy_input = *input;
        //change the fields [3] name to "intensity"
        // copy_input.fields[3].name = "intensity";
        
        
        pcl::fromROSMsg(*input, *cloud);
        // pcl::fromROSMsg(copy_input, *cloud);



        // Perform RANSAC plane segmentation
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.5);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);


        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);

        // Publish the model coefficients
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_plane, output);
        output.header.frame_id = input->header.frame_id;
        pub.publish(output);
    }

private:

    ros::NodeHandle nh;

    std::string input_topic;
    std::string output_topic;

    ros::Subscriber sub;
    ros::Publisher pub;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "ground_plane_segmentation");

    // Create our segmenter object
    GroundPlaneSegmentation segmenter;

    // Spin
    ros::spin();
}
