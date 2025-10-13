#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

#include "rclcpp/rclcpp.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include <filters/filter_chain.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

//TF
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PCD2PGM : public rclcpp::Node
{
public:
    PCD2PGM(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    void loadPCDFile(const std::string & filename);
    void publishPCD();
    void passThroughFilter(double thre_low, double thre_high, bool flag_in);
    void radiusOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count);
    void PCD2GridMap();
private:
    float thre_low;
    float thre_high;
    float radius;
    float thre_count;
    bool is_negative;
    std::string pcd_path_{};
    std::string config_path_{};
    std::string mapFrameId_{};
    std::string filterChainParametersName_{};
    rclcpp::Node::SharedPtr node_{};
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPublisher_;


    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_pass_through_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_radius_;

    grid_map::GridMapPclLoader gridMapPclLoader;
    grid_map::GridMap map_;

    filters::FilterChain<grid_map::GridMap> filterChain_;

};
