#include "pcd2pgm/pcd2pgm.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/io/pcd_io.h>
#include "pcl/common/transforms.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include <pcl/filters/conditional_removal.h>

PCD2PGM::PCD2PGM(const rclcpp::NodeOptions& options) : Node("pcd2pgm_node"),  map_(grid_map::GridMap({"elevation"})),
  gridMapPclLoader(this->get_logger()),
  filterChain_("grid_map::GridMap")
{
    declare_parameter("thre_low","");
    declare_parameter("thre_high","");
    declare_parameter("is_negative","false");
    declare_parameter("radius","");
    declare_parameter("thre_count","");
    declare_parameter("pcd_path","/ros2_ws/src/COD_NAV_NEXT/map/map.pcd");
    declare_parameter("config_path","");

    get_parameter("pcd_path",pcd_path_);
    get_parameter("thre_low",thre_low);
    get_parameter("thre_high",thre_high);
    get_parameter("radius",radius);
    get_parameter("thre_count",thre_count);
    get_parameter("is_negative",is_negative);
    get_parameter("config_patth",config_path_);

    pcd_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    gridMapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());
    loadPCDFile(pcd_path_);
    passThroughFilter(thre_low,thre_high,is_negative);
    radiusOutlierFilter(cloud_after_pass_through_,radius,thre_count);
}
void PCD2PGM::loadPCDFile(const std::string& filepath)
{
    pcl::io::loadPCDFile<pcl::PointXYZ>(filepath,*pcd_);

}

void PCD2PGM::passThroughFilter(double thre_low, double thre_high, bool flag_in)
{
    auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcd_);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(thre_low, thre_high);
    passthrough.setNegative(flag_in);
    passthrough.filter(*filtered_cloud);

    cloud_after_pass_through_ = filtered_cloud;
    RCLCPP_INFO(
      get_logger(), "After PassThrough filtering: %lu points",
      cloud_after_pass_through_->points.size());
}

void PCD2PGM::radiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, double radius, int thre_count)
{
    auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
    radius_outlier.setInputCloud(input_cloud);
    radius_outlier.setRadiusSearch(radius);
    radius_outlier.setMinNeighborsInRadius(thre_count);
    radius_outlier.filter(*filtered_cloud);

    cloud_after_radius_ = filtered_cloud;
    RCLCPP_INFO(
      get_logger(), "After RadiusOutlier filtering: %lu points", cloud_after_radius_->points.size());
}
void PCD2PGM::PCD2GridMap()
{
    gridMapPclLoader.setInputCloud(cloud_after_radius_);
    gridMapPclLoader.preProcessInputCloud();
    gridMapPclLoader.initializeGridMapGeometryFromInputCloud();
    gridMapPclLoader.addLayerFromInputCloud("elevation");
    map_ = gridMapPclLoader.getGridMap();
    map_.setFrameId(mapFrameId_);
    grid_map::GridMap outputMap;
    if (!filterChain_.update(map_, outputMap))
    {
        RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
        return;
    }
    auto message = grid_map::GridMapRosConverter::toMessage(outputMap);
    message->header.stamp = this->now();
    gridMapPublisher_->publish(std::move(message));
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCD2PGM>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}