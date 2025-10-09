#include "pcd2pgm/pcd2pgm.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/io/pcd_io.h>
#include "pcl/common/transforms.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"

PCD2PGM::PCD2PGM(const rclcpp::NodeOptions& options) : Node("pcd2pgm_node")
{
    pcd_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    loadPCDFile(pcd_path_);
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCD2PGM>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}