#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

std::string output_type;
std::string lidar_topic = "/point_cloud"; 

// VLP-16 
int N_SCAN = 16;
int Horizon_SCAN = 1800;    

static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPC;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPC;

template<typename T>
void publish_points(T &new_pc, const sensor_msgs::msg::PointCloud2 &old_msg) {
    // pc properties
    new_pc->is_dense = true;

    // publish
    sensor_msgs::msg::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*new_pc, pc_new_msg);
    pc_new_msg.header = old_msg.header;
    pc_new_msg.header.frame_id = "velodyne";
    pc_new_msg.header.stamp = old_msg.header.stamp;
    pubPC->publish(pc_new_msg);
}

void lidar_handle(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg_pointer) {
    sensor_msgs::msg::PointCloud2 pc_msg;
    pc_msg = *pc_msg_pointer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<PointXYZIRT>::Ptr pc_new(new pcl::PointCloud<PointXYZIRT>());
    pcl::fromROSMsg(pc_msg, *pc);

    // to new pointcloud
    for (int point_id = 0; point_id < pc->points.size(); ++point_id) {

        PointXYZIRT new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        new_point.intensity = 0; 

        //16 ring. The range of index is 0~15. Up to Down.
        float ang_bottom = 15.0+0.1;
        float ang_res_y = 2;
        float verticalAngle = atan2(new_point.z, sqrt(new_point.x * new_point.x + new_point.y * new_point.y)) * 180 / M_PI;
        float rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        
        new_point.ring = int(rowIdn);
        new_point.time = (point_id / N_SCAN)*0.1/Horizon_SCAN ;

        pc_new->points.push_back(new_point);
    }
    publish_points(pc_new, pc_msg);

}

int main(int argc, char **argv) {
    // ros::init(argc, argv, "rs_converter");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("converter");

    subPC = node->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, 10, lidar_handle);
    pubPC = node->create_publisher<sensor_msgs::msg::PointCloud2>("/points_raw", 10);

    RCLCPP_INFO(node->get_logger(), "Listening to lidar topic ......");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}