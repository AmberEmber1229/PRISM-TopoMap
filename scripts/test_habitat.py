import habitat_sim
import cv2
import numpy as np

# 创建一个简单的配置
backend_cfg = habitat_sim.SimulatorConfiguration()
# 注意：这里需要一个真实的 .glb 文件路径，请修改！
backend_cfg.scene_id = "/home/tom/prism_data/sim/mp3d_toposlam_validation_scenes/2n8kARJN3HM/2n8kARJN3HM.glb"

agent_cfg = habitat_sim.agent.AgentConfiguration()
sensor_spec = habitat_sim.SensorSpec()
sensor_spec.resolution = [480, 640]
sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
agent_cfg.sensor_specifications = [sensor_spec]

cfg = habitat_sim.Configuration(backend_cfg, [agent_cfg])

try:
    sim = habitat_sim.Simulator(cfg)
    obs = sim.reset()
    
    # ▼▼▼ 修改部分开始 ▼▼▼
    print("\n🔍 观测数据包含以下 Key:")
    print(obs.keys())  # 打印出实际存在的传感器名字
    
    # 自动寻找第一个彩色相机的数据
    rgb_key = None
    for k in obs:
        if "rgb" in k or "color" in k:  # 尝试模糊匹配
            rgb_key = k
            break
            
    if rgb_key:
        print(f"✅ 找到彩色相机数据: {rgb_key}")
        # 注意：OpenCV 保存图片需要 BGR 格式，而 Habitat 返回 RGB
        # 所以我们需要把 [..., :3] 翻转一下颜色通道
        img_bgr = obs[rgb_key][..., 0:3][..., ::-1] 
        cv2.imwrite("test_habitat_render.png", img_bgr)
        print("✅ 图片已保存为 test_habitat_render.png")
    else:
        print("❌ 未找到彩色相机数据 (RGB/Color)")
    # ▲▲▲ 修改部分结束 ▲▲▲
    
    sim.close()
except Exception as e:
    print(f"❌ 验证失败: {e}")
    import traceback
    traceback.print_exc()