# 本仓库的目的

1.把开源机械臂dummy如何接入moveit2相关代码和模型开源出来，解决这块资料缺乏的痛点 <br>
2.持续基于moveit2上添加机械臂的相关上层应用 <br>

# 已发布内容

1.dummy-ros2_description：使用fusion360导出的URDF，导出脚本可以看我另外一个项目 <br>
2.dummy_moveit_config：通过MoveIt Setup Assistant配置输出的相关move group配置，具体参考木子的项目和moveit2官方文档 <br>
3.dummy_controller：打通moveit2实现了与真实机械臂dummy的上下位机调用，结合了dummy项目自带的ref_tool的调用 <br>
4.dummy_server：实现了通过python代码调用moveit2的行动规划相关功能的调用 <br>
5.fusion2urdf-ros2：为用fusion设计的模型提供一个导出urdf模型的脚本，并支持ros2和新版fusion <br>

# 准备更新的内容

1.planning scene的障碍物设置 <br>
2.dummy与d435的手眼标定过程 <br>
3.夹爪的添加与控制 <br>

# 引用相关仓库

> peng-zhihui大神开源的dummy机械臂：https://github.com/peng-zhihui/Dummy-Robot.git [here](https://github.com/peng-zhihui/Dummy-Robot.git)!

> 木子改良版本的机械臂：https://gitee.com/switchpi/dummy.git [here](https://gitee.com/switchpi/dummy.git)!

> AndrejOrsula开源的调用moveit2的python工具库：https://github.com/AndrejOrsula/pymoveit2.git [here](https://github.com/AndrejOrsula/pymoveit2.git)!

> syuntoku14开源的fusion导出urdf工具：https://github.com/syuntoku14/fusion2urdf [here](https://github.com/syuntoku14/fusion2urdf)!

# 声明

本仓库遵循相关开源项目的开源协议，并不会把相关代码用于商业用途


