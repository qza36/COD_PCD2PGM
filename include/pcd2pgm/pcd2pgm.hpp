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
    void radiusOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count);
private:
    float thre_low;
    float thre_high;
    float radius;
    float thre_count;
    bool is_negative;
    std::string pcd_path_{};
    rclcpp::Node::SharedPtr node_{};

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_pass_through_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_radius_;
};
