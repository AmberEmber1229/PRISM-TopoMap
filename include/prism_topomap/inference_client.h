/**
 * @file inference_client.h
 * @brief Python 推理服务的 ROS Service 客户端封装
 *
 * 封装与 Python inference_service_node 的两个 Service 通信:
 *   1. /prism/get_descriptor      — 位置识别描述符提取
 *   2. /prism/grid_registration   — 栅格配准
 */
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <cstdint>
#include <vector>
#include <string>

// catkin 自动生成的 Service 头文件
#include <prism_topomap/GetDescriptor.h>
#include <prism_topomap/GridRegistration.h>
#include "prism_topomap/utils.h"

namespace prism_topomap {

class InferenceClient {
public:
    /**
     * @brief 构造函数
     * @param nh ROS NodeHandle
     * @param descriptor_service_name 描述符提取 Service 名称
     * @param registration_service_name 栅格配准 Service 名称
     */
    InferenceClient(ros::NodeHandle& nh,
                    const std::string& descriptor_service_name = "/prism/get_descriptor",
                    const std::string& registration_service_name = "/prism/grid_registration");

    /**
     * @brief 等待 Python 推理服务就绪
     * @param timeout_sec 超时时间 (秒)
     * @return 是否成功连接
     */
    bool waitForServices(double timeout_sec = 30.0);

    void setTraceConfig(const FlowTraceConfig& config) { trace_config_ = config; }
    void setTraceContext(int frame_id, bool detailed) {
        trace_frame_id_ = frame_id;
        trace_detailed_ = detailed;
    }

    // ========================================================================
    // 描述符提取
    // ========================================================================

    /// 描述符提取结果
    struct DescriptorResult {
        bool success;
        std::vector<float> descriptor;
        double elapsed_ms = 0.0;
    };

    /**
     * @brief 调用 Python 服务提取位置识别描述符
     *
     * @param cloud_msg       ROS PointCloud2 消息
     * @param has_image_front 是否有前视图像
     * @param has_image_back  是否有后视图像
     * @param image_front     前视图像消息
     * @param image_back      后视图像消息
     * @param quantization_size MinkowskiEngine 量化尺寸
     * @return 描述符结果
     */
    DescriptorResult getDescriptor(
        const sensor_msgs::PointCloud2& cloud_msg,
        bool has_image_front,
        bool has_image_back,
        const sensor_msgs::Image& image_front,
        const sensor_msgs::Image& image_back,
        float quantization_size);

    // ========================================================================
    // 栅格配准
    // ========================================================================

    /// 配准结果
    struct RegistrationResult {
        bool success;
        double score;
        double trans_i, trans_j, rot_angle;
        double elapsed_ms = 0.0;
    };

    /**
     * @brief 调用 Python 服务进行栅格配准
     *
     * @param ref_grid          参考栅格 (occupancy层, CV_8U)
     * @param cand_grid         候选栅格 (occupancy层, CV_8U)
     * @param registration_type "localization" 或 "inline"
     * @return 配准结果
     */
    RegistrationResult gridRegistration(
        const cv::Mat& ref_grid,
        const cv::Mat& cand_grid,
        const std::string& registration_type);

private:
    void resetDescriptorClient();
    void resetRegistrationClient();

    ros::NodeHandle nh_;
    std::string descriptor_service_name_;
    std::string registration_service_name_;
    ros::ServiceClient descriptor_client_;
    ros::ServiceClient registration_client_;
    uint64_t descriptor_call_count_ = 0;
    FlowTraceConfig trace_config_;
    int trace_frame_id_ = -1;
    bool trace_detailed_ = false;
};

} // namespace prism_topomap
