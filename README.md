🚀 PX4无人机控制库使用指南
🌐 环境介绍
本代码库专为PX4无人机仿真环境和真机环境开发设计，提供灵活的开发接口。


📥 仿真环境获取
如需获取完整仿真环境，请扫码联系：

微信：18873833517

🛠️ 使用说明
🖥️ 仿真模式

# 创建对象时选择模式
drone = DroneControl(mode='sim')       # 调试信息实时输出
drone = DroneControl(mode='sim_log')   # 调试信息静默模式
✅ ​适用场景​：

适用于iris_0开头的mavros话题系统

✈️ 实机模式

# 创建对象时选择模式
drone = DroneControl(mode='real')      # 调试信息实时输出
drone = DroneControl(mode='real_log')  # 调试信息静默模式
✅ ​适用场景​：

适用于标准mavros话题系统

🔧 环境依赖
必须安装组件
组件

安装方式

cartographer

GitHub仓库

movebase

GitHub仓库

其他依赖
请根据项目需求自行安装相关依赖项

⚠️ 注意事项
🔹 本库集成了RKNN的YOLOv8模型

🔹 ​无YOLOv8组件不影响核心功能使用​

🔹 建议使用Python 3.8+环境
