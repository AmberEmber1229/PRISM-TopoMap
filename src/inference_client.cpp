/**
 * @file inference_client.cpp
 * @brief Python 推理服务客户端的实现
 */
#include "prism_topomap/inference_client.h"
#include <ros/ros.h>

namespace prism_topomap {

// ============================================================================
// 构造函数
// ============================================================================
InferenceClient::InferenceClient(ros::NodeHandle& nh,
                                 const std::string& descriptor_service_name,
                                 const std::string& registration_service_name) {
    // 创建 Service 客户端 (persistent = true, 避免每次调用都重新连接)
    descriptor_client_ = nh.serviceClient<prism_topomap::GetDescriptor>(
        descriptor_service_name, true);
    registration_client_ = nh.serviceClient<prism_topomap::GridRegistration>(
        registration_service_name, true);
}

// ============================================================================
// waitForServices: 等待 Python 服务就绪
// ============================================================================
bool InferenceClient::waitForServices(double timeout_sec) {
    ROS_INFO("Waiting for Python inference services...");

    ros::Duration timeout(timeout_sec);

    bool desc_ok = descriptor_client_.waitForExistence(timeout);
    if (!desc_ok) {
        ROS_ERROR("Descriptor service [%s] not available!",
                  descriptor_client_.getService().c_str());
        return false;
    }
    ROS_INFO("Descriptor service is ready.");

    bool reg_ok = registration_client_.waitForExistence(timeout);
    if (!reg_ok) {
        ROS_ERROR("Registration service [%s] not available!",
                  registration_client_.getService().c_str());
        return false;
    }
    ROS_INFO("Registration service is ready.");
    ROS_INFO("All Python inference services are ready.");

    return true;
}

// ============================================================================
// getDescriptor: 调用 Python 服务提取描述符
// ============================================================================
InferenceClient::DescriptorResult InferenceClient::getDescriptor(
    const sensor_msgs::PointCloud2& cloud_msg,
    bool has_image_front,
    bool has_image_back,
    const sensor_msgs::Image& image_front,
    const sensor_msgs::Image& image_back,
    float quantization_size) {

    DescriptorResult result;
    result.success = false;

    prism_topomap::GetDescriptor srv;
    srv.request.pointcloud = cloud_msg;
    srv.request.has_image_front = has_image_front;
    srv.request.has_image_back = has_image_back;
    if (has_image_front) {
        srv.request.image_front = image_front;
    }
    if (has_image_back) {
        srv.request.image_back = image_back;
    }
    srv.request.quantization_size = quantization_size;

    if (descriptor_client_.call(srv)) {
        result.success = srv.response.success;
        if (result.success) {
            result.descriptor = srv.response.descriptor;
        }
    } else {
        ROS_WARN("GetDescriptor service call failed!");
        // 尝试重连
        descriptor_client_ = ros::NodeHandle().serviceClient<prism_topomap::GetDescriptor>(
            descriptor_client_.getService(), true);
    }

    return result;
}

// ============================================================================
// gridRegistration: 调用 Python 服务进行栅格配准
// ============================================================================
InferenceClient::RegistrationResult InferenceClient::gridRegistration(
    const cv::Mat& ref_grid,
    const cv::Mat& cand_grid,
    const std::string& registration_type) {

    RegistrationResult result;
    result.success = false;
    result.score = 0.0;
    result.trans_i = 0.0;
    result.trans_j = 0.0;
    result.rot_angle = 0.0;

    prism_topomap::GridRegistration srv;

    // 设置栅格尺寸
    srv.request.grid_height = ref_grid.rows;
    srv.request.grid_width = ref_grid.cols;

    // 将 cv::Mat 展平为 uint8 数组
    // ref_grid: 按行展平
    srv.request.ref_grid.resize(ref_grid.rows * ref_grid.cols);
    for (int r = 0; r < ref_grid.rows; ++r) {
        for (int c = 0; c < ref_grid.cols; ++c) {
            srv.request.ref_grid[r * ref_grid.cols + c] = ref_grid.at<uint8_t>(r, c);
        }
    }

    // cand_grid: 按行展平
    srv.request.cand_grid.resize(cand_grid.rows * cand_grid.cols);
    for (int r = 0; r < cand_grid.rows; ++r) {
        for (int c = 0; c < cand_grid.cols; ++c) {
            srv.request.cand_grid[r * cand_grid.cols + c] = cand_grid.at<uint8_t>(r, c);
        }
    }

    srv.request.registration_type = registration_type;

    if (registration_client_.call(srv)) {
        result.success = srv.response.success;
        result.score = srv.response.score;
        result.trans_i = srv.response.trans_i;
        result.trans_j = srv.response.trans_j;
        result.rot_angle = srv.response.rot_angle;
    } else {
        ROS_WARN("GridRegistration service call failed!");
        // 尝试重连
        registration_client_ = ros::NodeHandle().serviceClient<prism_topomap::GridRegistration>(
            registration_client_.getService(), true);
    }

    return result;
}

} // namespace prism_topomap
