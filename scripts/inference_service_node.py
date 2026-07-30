#!/usr/bin/env python
"""
PRISM-TopoMap Python 推理服务节点
inference_service_node.py

提供两个 ROS Service:
  /prism/get_descriptor      - 位置识别描述符提取
  /prism/grid_registration   - 栅格到栅格配准

启动后自动加载模型权重, 等待 C++ 主节点的 Service 调用
"""

import rospy
import numpy as np
import torch
import yaml
import os
import sys
import time

# 将脚本所在目录加入 Python 路径, 确保能找到同目录下的 models.py 等模块
# 注意: 必须用 append 而非 insert(0), 因为 scripts/ 下有 prism_topomap.py
# 与 catkin 生成的 prism_topomap 包同名, insert(0) 会导致命名冲突
_scripts_dir = os.path.dirname(os.path.abspath(__file__))
if _scripts_dir not in sys.path:
    sys.path.append(_scripts_dir)

# ROS 消息
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2

# 自定义 Service (catkin 自动生成)
from prism_topomap.srv import GetDescriptor, GetDescriptorResponse
from prism_topomap.srv import GridRegistration, GridRegistrationResponse

# MinkowskiEngine (用于稀疏点云量化)
try:
    import MinkowskiEngine as ME
    HAS_ME = True
except ImportError:
    HAS_ME = False
    rospy.logwarn("MinkowskiEngine 未安装, 点云描述符提取不可用")


class InferenceServiceNode:
    """
    Python 推理服务节点

    加载并管理:
    1. 位置识别模型 (MinkLoc3D / MSSPlace)
    2. 两套配准模型:
       - registration_pipeline: 全局定位配准
       - inline_registration_pipeline: 沿边配准
    """

    def __init__(self):
        rospy.init_node('prism_inference_service')
        rospy.loginfo("=== PRISM 推理服务节点初始化中... ===")

        # ================================================================
        # 1. 加载配置
        # ================================================================
        config_file = rospy.get_param('~config_file', 'scout_rosbag.yaml')
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(pkg_dir, 'config', config_file)
        rospy.loginfo(f"配置文件路径: {config_path}")

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Optional end-to-end data-flow tracing. All new detailed output is off
        # unless explicitly enabled by launch/private parameters.
        self.trace_data_flow = rospy.get_param('~trace_data_flow', False)
        self.trace_every_n_processed_frames = max(
            1, int(rospy.get_param('~trace_every_n_processed_frames', 1)))
        self.trace_descriptor_head_size = max(
            0, int(rospy.get_param('~trace_descriptor_head_size', 4)))
        self.trace_registration_candidates = rospy.get_param(
            '~trace_registration_candidates', True)
        self.descriptor_request_count = 0
        self.registration_request_count = 0

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo(f"推理设备: {self.device}")

        # ================================================================
        # 2. 加载位置识别模型
        # ================================================================
        from models import get_place_recognition_model
        # 原 API: get_place_recognition_model(config) 接受 place_recognition 子字典
        # 返回 (model, faiss_index)
        pr_config = self.config['place_recognition']
        self.place_recognition_model, _ = get_place_recognition_model(pr_config)
        # 注意: FAISS 索引已在 C++ 端管理, Python 端返回的 index 不使用
        self.pointcloud_quantization_size = pr_config.get(
            'pointcloud_quantization_size', 0.5)
        rospy.loginfo("位置识别模型加载完成")

        # ================================================================
        # 3. 加载配准模型
        # ================================================================
        from models import get_registration_model

        # 3a. 全局定位配准模型 (对应 config['scan_matching'])
        reg_config = dict(self.config['scan_matching'])
        # 原代码: registration_config['voxel_downsample_size'] = grid_config['resolution']
        grid_config = self.config['local_occupancy_grid']
        reg_config['voxel_downsample_size'] = grid_config['resolution']
        self.registration_pipeline = get_registration_model(reg_config)
        self.registration_score_threshold = reg_config.get('score_threshold', 0.6)
        rospy.loginfo("全局定位配准模型加载完成")

        # 3b. 沿边配准模型 (对应 config['scan_matching_along_edge'])
        inline_reg_config = dict(self.config['scan_matching_along_edge'])
        inline_reg_config['voxel_downsample_size'] = grid_config['resolution']
        self.inline_registration_pipeline = get_registration_model(inline_reg_config)
        self.inline_registration_score_threshold = inline_reg_config.get('score_threshold', 0.5)
        rospy.loginfo("沿边配准模型加载完成")

        # ================================================================
        # 4. 注册 ROS Service
        # ================================================================
        self.descriptor_service = rospy.Service(
            '/prism/get_descriptor',
            GetDescriptor,
            self.handle_get_descriptor
        )
        self.registration_service = rospy.Service(
            '/prism/grid_registration',
            GridRegistration,
            self.handle_grid_registration
        )

        rospy.loginfo("=== PRISM 推理服务节点初始化完成 ===")
        rospy.loginfo("  - 描述符提取服务: /prism/get_descriptor")
        rospy.loginfo("  - 栅格配准服务: /prism/grid_registration")
        if self.trace_data_flow:
            rospy.loginfo(
                "[FLOW][STAGE=DESCRIPTOR_PY] trace_enabled=true "
                "every_n_processed_frames=%d descriptor_head_size=%d "
                "registration_candidates=%s",
                self.trace_every_n_processed_frames,
                self.trace_descriptor_head_size,
                str(self.trace_registration_candidates).lower())

    # ====================================================================
    # 描述符提取 Service Handler
    # ====================================================================
    def handle_get_descriptor(self, req):
        """
        处理描述符提取请求

        输入: PointCloud2 + 可选 Image
        输出: float[] descriptor

        流程:
        1. 解析 ROS PointCloud2 → numpy array
        2. MinkowskiEngine 稀疏量化
        3. 可选图像预处理
        4. 模型前向推理
        5. 返回描述符
        """
        resp = GetDescriptorResponse()
        resp.success = False
        self.descriptor_request_count += 1
        request_id = self.descriptor_request_count
        trace_detailed = (
            self.trace_data_flow and
            (request_id - 1) % self.trace_every_n_processed_frames == 0)
        request_start = time.perf_counter()
        forward_ms = 0.0
        raw_points = int(req.pointcloud.width * req.pointcloud.height)
        finite_points = 0
        quantized_points = 0
        front_shape = "NONE"
        back_shape = "NONE"

        if self.trace_data_flow:
            rospy.loginfo(
                "[FLOW][FRAME=SERVICE-%d][STAMP=%.9f]"
                "[STAGE=DESCRIPTOR_PY] action=HANDLER_BEGIN "
                "raw_points=%d quantization=%.6f",
                request_id, req.pointcloud.header.stamp.to_sec(),
                raw_points, req.quantization_size)

        try:
            # 1. 解析点云
            cloud_array = []
            for point in pc2.read_points(req.pointcloud,
                                          field_names=('x', 'y', 'z'),
                                          skip_nans=True):
                cloud_array.append([point[0], point[1], point[2]])

            if len(cloud_array) == 0:
                rospy.logwarn("收到空点云, 跳过描述符提取")
                if self.trace_data_flow:
                    rospy.loginfo(
                        "[FLOW][FRAME=SERVICE-%d][STAMP=%.9f]"
                        "[STAGE=DESCRIPTOR_PY] result=EMPTY_CLOUD "
                        "raw_points=%d finite_points=0 elapsed_ms=%.3f",
                        request_id, req.pointcloud.header.stamp.to_sec(),
                        raw_points,
                        (time.perf_counter() - request_start) * 1000.0)
                return resp

            cloud_np = np.array(cloud_array, dtype=np.float32)
            finite_points = int(cloud_np.shape[0])

            # 2. 构建模型输入
            input_data = {
                'pointcloud_lidar_coords': torch.Tensor(cloud_np[:, :3]).to(self.device),
                'pointcloud_lidar_feats': torch.ones((cloud_np.shape[0], 1)).to(self.device),
            }

            # 3. 可选图像
            if req.has_image_front:
                img_front = self._ros_image_to_tensor(req.image_front)
                if img_front is not None:
                    input_data['image_front'] = img_front
                    front_shape = "x".join(str(v) for v in img_front.shape)

            if req.has_image_back:
                img_back = self._ros_image_to_tensor(req.image_back)
                if img_back is not None:
                    input_data['image_back'] = img_back
                    back_shape = "x".join(str(v) for v in img_back.shape)

            # 4. 预处理 (MinkowskiEngine 稀疏量化)
            batch = self._preprocess_input(input_data, req.quantization_size)
            quantized_points = int(batch["pointclouds_lidar_coords"].shape[0])

            # 5. 模型推理
            forward_start = time.perf_counter()
            with torch.no_grad():
                output = self.place_recognition_model(batch)
                descriptor = output["final_descriptor"].detach().cpu().numpy()
            forward_ms = (time.perf_counter() - forward_start) * 1000.0

            # 确保是一维
            if len(descriptor.shape) > 1:
                descriptor = descriptor.flatten()

            resp.success = True
            resp.descriptor = descriptor.tolist()
            rospy.loginfo("[INFER-PY] getDescriptor OK, dim=%d", len(resp.descriptor))
            if trace_detailed:
                head_size = min(
                    self.trace_descriptor_head_size, len(resp.descriptor))
                head = ",".join(
                    "{:.6f}".format(value)
                    for value in resp.descriptor[:head_size])
                rospy.loginfo(
                    "[FLOW][FRAME=SERVICE-%d][STAMP=%.9f]"
                    "[STAGE=DESCRIPTOR_PY] result=OK raw_points=%d "
                    "finite_points=%d invalid_points=%d quantization=%.6f "
                    "quantized_points=%d front_image=%s front_shape=%s "
                    "back_image=%s back_shape=%s model=%s device=%s "
                    "descriptor_dim=%d descriptor_l2=%.6f "
                    "descriptor_head=[%s] forward_ms=%.3f elapsed_ms=%.3f",
                    request_id, req.pointcloud.header.stamp.to_sec(),
                    raw_points, finite_points,
                    raw_points - finite_points, req.quantization_size,
                    quantized_points,
                    str(req.has_image_front).lower(), front_shape,
                    str(req.has_image_back).lower(), back_shape,
                    type(self.place_recognition_model).__name__, str(self.device),
                    len(resp.descriptor),
                    float(np.linalg.norm(descriptor)), head, forward_ms,
                    (time.perf_counter() - request_start) * 1000.0)

        except Exception as e:
            rospy.logerr(f"描述符提取异常: {str(e)}")
            if self.trace_data_flow:
                rospy.loginfo(
                    "[FLOW][FRAME=SERVICE-%d][STAMP=%.9f]"
                    "[STAGE=DESCRIPTOR_PY] result=EXCEPTION "
                    "raw_points=%d finite_points=%d quantized_points=%d "
                    "error=%s forward_ms=%.3f elapsed_ms=%.3f",
                    request_id, req.pointcloud.header.stamp.to_sec(),
                    raw_points, finite_points, quantized_points,
                    str(e).replace(" ", "_"), forward_ms,
                    (time.perf_counter() - request_start) * 1000.0)
            import traceback
            traceback.print_exc()

        return resp

    # ====================================================================
    # 栅格配准 Service Handler
    # ====================================================================
    def handle_grid_registration(self, req):
        """
        处理栅格配准请求

        输入: ref_grid + cand_grid + registration_type
        输出: success, score, trans_i, trans_j, rot_angle

        流程:
        1. 反序列化栅格数据
        2. 转为 torch.Tensor
        3. 选择配准 Pipeline
        4. 推理
        5. 返回结果
        """
        resp = GridRegistrationResponse()
        resp.success = False
        resp.score = 0.0
        resp.trans_i = 0.0
        resp.trans_j = 0.0
        resp.rot_angle = 0.0
        self.registration_request_count += 1
        request_id = self.registration_request_count
        request_start = time.perf_counter()
        trace_registration = (
            self.trace_data_flow and self.trace_registration_candidates)
        ref_nonzero = 0
        cand_nonzero = 0

        try:
            # 1. 反序列化栅格
            h = req.grid_height
            w = req.grid_width
            
            # ROS Noetic (Python 3) 中 uint8[] 会被反序列化为 bytes 字节串
            # 所以不能直接用 np.array(..., dtype)
            if isinstance(req.ref_grid, bytes):
                ref_grid_np = np.frombuffer(req.ref_grid, dtype=np.uint8).reshape(h, w)
            else:
                ref_grid_np = np.array(req.ref_grid, dtype=np.uint8).reshape(h, w)
                
            if isinstance(req.cand_grid, bytes):
                cand_grid_np = np.frombuffer(req.cand_grid, dtype=np.uint8).reshape(h, w)
            else:
                cand_grid_np = np.array(req.cand_grid, dtype=np.uint8).reshape(h, w)
            ref_nonzero = int(np.count_nonzero(ref_grid_np))
            cand_nonzero = int(np.count_nonzero(cand_grid_np))

            # 2. 转为 Tensor
            ref_grid_tensor = torch.Tensor(ref_grid_np.astype(np.float32)).to(self.device)
            cand_grid_tensor = torch.Tensor(cand_grid_np.astype(np.float32)).to(self.device)

            # 3. 选择 Pipeline
            if req.registration_type == "inline":
                pipeline = self.inline_registration_pipeline
            else:
                pipeline = self.registration_pipeline

            # 4. 推理
            with torch.no_grad():
                transform, score = pipeline.infer(
                    ref_grid_tensor, cand_grid_tensor, verbose=False)

            if transform is None:
                # 配准失败（比如没有任何重叠或匹配点）
                resp.success = False
                resp.score = 0.0
                if trace_registration:
                    rospy.loginfo(
                        "[FLOW][FRAME=SERVICE-REG-%d][STAMP=UNAVAILABLE]"
                        "[STAGE=REGISTRATION_PY] candidate=UNKNOWN type=%s "
                        "grid=%dx%d ref_nonzero=%d cand_nonzero=%d "
                        "service_success=false result=NO_TRANSFORM "
                        "elapsed_ms=%.3f",
                        request_id, req.registration_type, h, w,
                        ref_nonzero, cand_nonzero,
                        (time.perf_counter() - request_start) * 1000.0)
                return resp

            resp.success = True
            resp.score = float(score)
            resp.trans_i = float(transform[0])
            resp.trans_j = float(transform[1])
            resp.rot_angle = float(transform[2])
            rospy.loginfo("[INFER-PY] gridRegistration type=%s score=%.3f "
                          "trans=(%.2f,%.2f,%.3f rad)",
                          req.registration_type, resp.score,
                          resp.trans_i, resp.trans_j, resp.rot_angle)
            if trace_registration:
                rospy.loginfo(
                    "[FLOW][FRAME=SERVICE-REG-%d][STAMP=UNAVAILABLE]"
                    "[STAGE=REGISTRATION_PY] candidate=UNKNOWN type=%s "
                    "grid=%dx%d ref_nonzero=%d cand_nonzero=%d "
                    "service_success=true score=%.6f "
                    "pixel_transform=(%.6f,%.6f,%.6f) elapsed_ms=%.3f "
                    "result=OK",
                    request_id, req.registration_type, h, w,
                    ref_nonzero, cand_nonzero, resp.score,
                    resp.trans_i, resp.trans_j, resp.rot_angle,
                    (time.perf_counter() - request_start) * 1000.0)

        except Exception as e:
            rospy.logerr(f"栅格配准异常: {str(e)}")
            if self.trace_data_flow:
                rospy.loginfo(
                    "[FLOW][FRAME=SERVICE-REG-%d][STAMP=UNAVAILABLE]"
                    "[STAGE=REGISTRATION_PY] candidate=UNKNOWN type=%s "
                    "grid=%dx%d ref_nonzero=%d cand_nonzero=%d "
                    "service_success=false result=EXCEPTION error=%s "
                    "elapsed_ms=%.3f",
                    request_id, req.registration_type,
                    req.grid_height, req.grid_width,
                    ref_nonzero, cand_nonzero, str(e).replace(" ", "_"),
                    (time.perf_counter() - request_start) * 1000.0)
            import traceback
            traceback.print_exc()

        return resp

    # ====================================================================
    # 内部辅助方法
    # ====================================================================

    def _ros_image_to_tensor(self, img_msg):
        """ROS Image 消息 → PyTorch Tensor (C, H, W)"""
        try:
            from cv_bridge import CvBridge
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            # BGR → RGB, HWC → CHW
            rgb = cv_image[:, :, ::-1].copy()
            tensor = torch.Tensor(rgb).to(self.device)
            tensor = torch.permute(tensor, (2, 0, 1))
            return tensor
        except Exception as e:
            rospy.logwarn(f"图像转换失败: {str(e)}")
            return None

    def _preprocess_input(self, input_data, quantization_size=None):
        """
        预处理输入数据 (对应原 Python _preprocess_input)

        对点云进行 MinkowskiEngine 稀疏量化,
        对图像进行维度调整
        """
        if quantization_size is None:
            quantization_size = self.pointcloud_quantization_size

        out_dict = {}
        for key in input_data:
            if key.startswith("image_"):
                out_dict[f"images_{key[6:]}"] = input_data[key].unsqueeze(0)
            elif key.startswith("mask_"):
                out_dict[f"masks_{key[5:]}"] = input_data[key].unsqueeze(0)
            elif key == "pointcloud_lidar_coords":
                if HAS_ME:
                    quantized_coords, quantized_feats = ME.utils.sparse_quantize(
                        coordinates=input_data["pointcloud_lidar_coords"],
                        features=input_data["pointcloud_lidar_feats"],
                        quantization_size=quantization_size,
                    )
                    out_dict["pointclouds_lidar_coords"] = ME.utils.batched_coordinates(
                        [quantized_coords]).to(self.device)
                    out_dict["pointclouds_lidar_feats"] = quantized_feats.to(self.device)
                else:
                    rospy.logerr("MinkowskiEngine 未安装, 无法量化点云!")
                    raise RuntimeError("MinkowskiEngine not available")
        return out_dict

    def run(self):
        """主循环"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = InferenceServiceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"推理服务节点异常退出: {str(e)}")
        import traceback
        traceback.print_exc()
