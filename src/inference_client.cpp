/**
 * @file inference_client.cpp
 * @brief Python 推理服务客户端的实现
 */
#include "prism_topomap/inference_client.h"
#include <ros/ros.h>
#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace prism_topomap {

// ============================================================================
// 构造函数
// ============================================================================
InferenceClient::InferenceClient(ros::NodeHandle& nh,
                                 const std::string& descriptor_service_name,
                                 const std::string& registration_service_name)
    : nh_(nh),
      descriptor_service_name_(descriptor_service_name),
      registration_service_name_(registration_service_name) {
    resetDescriptorClient();
    resetRegistrationClient();
}

void InferenceClient::resetDescriptorClient() {
    if (descriptor_client_.isValid()) {
        descriptor_client_.shutdown();
    }
    // A fresh non-persistent connection avoids carrying a stale TCPROS
    // connection across a long delay between launch and the first rosbag
    // point cloud. The local connection setup is small compared with request
    // serialization and model inference, and can be measured in the trace.
    descriptor_client_ = nh_.serviceClient<prism_topomap::GetDescriptor>(
        descriptor_service_name_, false);
}

void InferenceClient::resetRegistrationClient() {
    if (registration_client_.isValid()) {
        registration_client_.shutdown();
    }
    // Registration can also remain unused while waiting for rosbag playback.
    // Use a fresh TCPROS connection per call so the first localization request
    // does not inherit a stale persistent connection.
    registration_client_ = nh_.serviceClient<prism_topomap::GridRegistration>(
        registration_service_name_, false);
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
    ROS_INFO("Descriptor service is ready (persistent=false, "
             "transport_retries=1).");

    bool reg_ok = registration_client_.waitForExistence(timeout);
    if (!reg_ok) {
        ROS_ERROR("Registration service [%s] not available!",
                  registration_client_.getService().c_str());
        return false;
    }
    ROS_INFO("Registration service is ready (persistent=false, "
             "transport_retries=1).");
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
    const auto call_start = std::chrono::steady_clock::now();
    const uint64_t call_id = ++descriptor_call_count_;

    if (trace_config_.enabled && trace_detailed_) {
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DESCRIPTOR] action=SERVICE_REQUEST "
                 "points=%u front_image=%s back_image=%s quantization=%.3f",
                 trace_frame_id_, cloud_msg.header.stamp.toSec(),
                 cloud_msg.width * cloud_msg.height,
                 has_image_front ? "true" : "false",
                 has_image_back ? "true" : "false",
                 quantization_size);
    }

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

    bool transport_success = false;
    int attempts = 0;
    constexpr int kMaxAttempts = 2;
    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        attempts = attempt;
        transport_success = descriptor_client_.call(srv);
        if (transport_success) {
            break;
        }

        ROS_WARN("[INFER] GetDescriptor transport failed "
                 "(call_id=%llu attempt=%d/%d service=%s exists=%s valid=%s)",
                 static_cast<unsigned long long>(call_id),
                 attempt, kMaxAttempts, descriptor_service_name_.c_str(),
                 descriptor_client_.exists() ? "true" : "false",
                 descriptor_client_.isValid() ? "true" : "false");

        if (attempt < kMaxAttempts) {
            resetDescriptorClient();
            const bool service_ready =
                descriptor_client_.exists() ||
                descriptor_client_.waitForExistence(ros::Duration(1.0));
            ROS_WARN("[INFER] GetDescriptor client recreated before retry "
                     "(call_id=%llu service_ready=%s)",
                     static_cast<unsigned long long>(call_id),
                     service_ready ? "true" : "false");
        }
    }

    if (transport_success) {
        result.success = srv.response.success;
        if (result.success) {
            result.descriptor = srv.response.descriptor;
            ROS_INFO_THROTTLE(5.0, "[INFER] getDescriptor OK, dim=%lu",
                              result.descriptor.size());
        } else {
            ROS_WARN_THROTTLE(5.0, "[INFER] getDescriptor returned success=false");
        }
    } else {
        ROS_ERROR("[INFER] GetDescriptor transport failed after %d attempts "
                  "(call_id=%llu service=%s)",
                  attempts, static_cast<unsigned long long>(call_id),
                  descriptor_service_name_.c_str());
    }

    result.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - call_start).count();

    if (trace_config_.enabled && (trace_detailed_ || !result.success)) {
        double norm_sq = 0.0;
        for (float value : result.descriptor) {
            norm_sq += static_cast<double>(value) * value;
        }
        std::ostringstream head;
        head << std::fixed << std::setprecision(5) << "[";
        const int head_size = std::max(0, trace_config_.descriptor_head_size);
        const size_t n = std::min(result.descriptor.size(), static_cast<size_t>(head_size));
        for (size_t i = 0; i < n; ++i) {
            if (i > 0) head << ",";
            head << result.descriptor[i];
        }
        if (result.descriptor.size() > n) head << ",...";
        head << "]";

        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DESCRIPTOR] action=SERVICE_RESULT "
                 "success=%s transport_success=%s call_id=%llu attempts=%d "
                 "elapsed_ms=%.3f dim=%lu l2_norm=%.6f head=%s",
                 trace_frame_id_, cloud_msg.header.stamp.toSec(),
                 result.success ? "true" : "false",
                 transport_success ? "true" : "false",
                 static_cast<unsigned long long>(call_id), attempts,
                 result.elapsed_ms,
                 result.descriptor.size(), std::sqrt(norm_sq), head.str().c_str());
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
    const auto call_start = std::chrono::steady_clock::now();

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

    bool transport_success = false;
    int attempts = 0;
    constexpr int kMaxAttempts = 2;
    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        attempts = attempt;
        transport_success = registration_client_.call(srv);
        if (transport_success) {
            break;
        }

        ROS_WARN("[INFER] GridRegistration transport failed "
                 "(type=%s attempt=%d/%d service=%s exists=%s valid=%s)",
                 registration_type.c_str(), attempt, kMaxAttempts,
                 registration_service_name_.c_str(),
                 registration_client_.exists() ? "true" : "false",
                 registration_client_.isValid() ? "true" : "false");

        if (attempt < kMaxAttempts) {
            resetRegistrationClient();
            const bool service_ready =
                registration_client_.exists() ||
                registration_client_.waitForExistence(ros::Duration(1.0));
            ROS_WARN("[INFER] GridRegistration client recreated before retry "
                     "(type=%s service_ready=%s)",
                     registration_type.c_str(),
                     service_ready ? "true" : "false");
        }
    }

    if (transport_success) {
        result.success = srv.response.success;
        result.score = srv.response.score;
        result.trans_i = srv.response.trans_i;
        result.trans_j = srv.response.trans_j;
        result.rot_angle = srv.response.rot_angle;
        if (result.success) {
            ROS_INFO("[INFER] gridRegistration type=%s score=%.3f "
                     "trans=(%.2f,%.2f,%.3f rad)",
                     registration_type.c_str(), result.score,
                     result.trans_i, result.trans_j, result.rot_angle);
        }
    } else {
        ROS_ERROR("[INFER] GridRegistration transport failed after %d attempts "
                  "(type=%s service=%s)",
                  attempts, registration_type.c_str(),
                  registration_service_name_.c_str());
    }

    result.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - call_start).count();
    if (trace_config_.enabled && (trace_detailed_ || !result.success)) {
        const char* trace_result = result.success
            ? "OK"
            : (transport_success ? "NO_TRANSFORM" : "TRANSPORT_FAILURE");
        if (result.success) {
            ROS_INFO("[FLOW][STAGE=REGISTRATION_SERVICE] action=SERVICE_RESULT "
                     "type=%s result=%s success=true transport_success=true "
                     "attempts=%d elapsed_ms=%.3f ref=%dx%d cand=%dx%d",
                     registration_type.c_str(), trace_result, attempts,
                     result.elapsed_ms, ref_grid.rows, ref_grid.cols,
                     cand_grid.rows, cand_grid.cols);
        } else {
            ROS_WARN("[FLOW][STAGE=REGISTRATION_SERVICE] action=SERVICE_RESULT "
                     "type=%s result=%s success=false transport_success=%s "
                     "attempts=%d elapsed_ms=%.3f ref=%dx%d cand=%dx%d",
                     registration_type.c_str(), trace_result,
                     transport_success ? "true" : "false", attempts,
                     result.elapsed_ms, ref_grid.rows, ref_grid.cols,
                     cand_grid.rows, cand_grid.cols);
        }
    }

    return result;
}

} // namespace prism_topomap
