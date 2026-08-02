# 本仓库的目的

1.把开源机械臂dummy如何接入moveit2相关代码和模型开源出来，解决这块资料缺乏的痛点 <br>
2.持续基于moveit2上添加机械臂的相关上层应用 <br>

# 已发布内容

1.dummy-ros2_description：使用fusion360导出的URDF，导出脚本可以看我另外一个项目 <br>
2.dummy_moveit_config：通过MoveIt Setup Assistant配置输出的相关move group配置，具体参考木子的项目和moveit2官方文档 <br>
3.dummy_controller：打通moveit2实现了与真实机械臂dummy的上下位机调用，结合了dummy项目自带的ref_tool的调用 <br>
4.dummy_server：实现了通过python代码调用moveit2的行动规划相关功能的调用 <br>
5.dummy_vision：dummy与d435的手眼标定过程，配置调试笔记见doc目录中《Dummy手眼标定笔记.pdf》 <br>

# moveit流式控制dummy

![Alt text](doc/moveit流式控制.jpg)

1.已经更新到项目中，可以通过moveit的servo流式发送工具坐标，moveit将实时逆解为J1到J6的关键坐标进行移动，过程非常丝滑，精度也非常不错 <br>
2.改动涉及dummy_controller和dummy_server两模块 <br>

### 流式控制服务启动与调试

1.启动moveit中的机械臂 <br>
ros2 launch dummy_moveit_config servo_streaming.launch.py <br>

2.启动moveit中的流式服务 <br>
cd ~/apps/dummy_ws <br>
python src/dummy_server/server/stream_api_server.py（如果控制没反应，重复启用两次即可） <br>

3.测试demo <br>
python src/dummy_server/server/stream_api_client_demo.py <br>

#### 注意：详细笔记见doc目录中《moveit流式控制笔记.md》
<br>

# 使用Lerobot训练dummy

![Alt text](doc/使用Lerobot训练dummy.jpg)

兄弟们，这块可以关注我另外一个开源项目，感谢感谢！
### https://github.com/hata8210/lerobot

<br>

# 准备更新的内容

1.dummy仿真强化学习 <br>
2.夹爪的添加与控制 <br>


# 引用相关仓库

> peng-zhihui大神开源的dummy机械臂：https://github.com/peng-zhihui/Dummy-Robot.git 

> 木子改良版本的机械臂：https://gitee.com/switchpi/dummy.git 

> AndrejOrsula开源的调用moveit2的python工具库：https://github.com/AndrejOrsula/pymoveit2.git 

> syuntoku14开源的fusion导出urdf工具：https://github.com/syuntoku14/fusion2urdf 

> Huggingface的Lerobot项目：https://github.com/huggingface/lerobot 

# 声明

本仓库遵循相关开源项目的开源协议，并不会把相关代码用于商业用途


