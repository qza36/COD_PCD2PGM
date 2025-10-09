#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

#include "rclcpp/rclcpp.hpp"

class PCD2PGM : public rclcpp::Node
{
public:
    PCD2PGM(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    void loadPCDFile(const std::string & filename);
    void publishPCD();
    void passThroughFilter(double thre_low, double thre_high, bool flag_in);
private:
    std::string pcd_path_{"/ros2_ws/src/COD_NAV_NEXT/map/map.pcd"};
    rclcpp::Node::SharedPtr node_{};

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_pass_through_;
};
