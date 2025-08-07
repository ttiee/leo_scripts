# 项目AI说明

本文件为AI使用的提示文档, 旨在让AI充分理解`leo_scripts`仓库中每个脚本的作用、所使用的ROS话题/服务/参数, 以及关键依赖关系。

## 1. 项目概览

该项目围绕移动机器人(可能使用ROS Navigation stack)和Dobot机械臂进行集成，实现地图导航、姿态修正、盒子拾取与放置、二维码识别、位姿记录等功能。所有脚本均使用Python编写, 并依赖ROS的`rospy`客户端库。

以下是各主要模块与其功能概述:

- **Nav_Api.py**: 机器人导航API, 提供获取当前位姿、设置导航目标、取消导航、获取导航状态以及设置目标容差等功能。
- **robot_correct_position.py**: 发布`/initialpose`话题以修正机器人位姿, 提供从当前位姿或指定位姿进行修正的两个函数。
- **robot_move_distance.py**: 使用`/cmd_vel`话题驱动底盘, 基于TF计算实际位移以实现定距离移动。
- **QRcode_scan.py**: 订阅`/usb_cam/image_raw`图像话题, 通过`pyzbar`解码获取二维码中的省份信息。
- **yaml_position_recorder.py**: 交互式记录机器人位姿到YAML文件, 支持加载模板并动态更新点位。
- **yaml_box_delivery.py**: 综合使用导航API、机械臂控制模块、二维码识别结果和位姿修正功能, 完成从货架取盒、识别目标省份并导航投放的流程。
- **Dobot/**: 机械臂相关脚本, 通过串口或SDK控制Dobot完成取放动作。
- **utils/**: 通用工具库, 包含按键监听、YAML读写等辅助函数。

## 2. ROS话题与服务

### 2.1 话题

| 脚本 | 发布话题 | 订阅/等待话题 | 消息类型 | 说明 |
|------|----------|---------------|----------|------|
| Nav_Api.py | `/move_base/goal`, `/move_base/cancel` | `robot_pose`, `/move_base/status` | `MoveBaseActionGoal`, `GoalID`, `Pose`, `GoalStatusArray` | 设置导航目标、取消目标并查询状态 |
| robot_correct_position.py | `/initialpose` | — | `PoseWithCovarianceStamped` | 将当前或指定位姿发布到AMCL进行姿态修正 |
| robot_move_distance.py | `/cmd_vel` | TF:`/odom_combined`→`/base_footprint` | `Twist` | 控制底盘线速度, 通过TF估算位移 |
| QRcode_scan.py | — | `/usb_cam/image_raw` | `Image` | 订阅摄像头图像并解析二维码 |
| test/check_linear_imu.py | `/cmd_vel` | — | `Twist` | 测试底盘线速度 |
| test/csv_multi_point_nav.py | — | `/move_base/status` | `GoalStatusArray` | 读取导航状态 |
| old/old_pub_province.py | `/qrcode_info` | `/usb_cam/image_raw` | `String`, `Image` | 发布二维码解码结果 |
| old/old_QRcode_scan.py | — | `/usb_cam/image_raw` | `Image` | 旧版二维码扫描脚本 |
| old/old_sub_province.py | — | `/qrcode_info` (等待消息) | `String` | 旧版省份信息订阅 |

除上述话题外, 各脚本还通过`rospy.set_param`与ROS参数服务器交互, 例如设置`/move_base/TebLocalPlannerROS/xy_goal_tolerance`和`/move_base/TebLocalPlannerROS/yaw_goal_tolerance`等导航参数。

### 2.2 服务

| 服务名称 | 服务类型 | 说明 |
|----------|----------|------|
| `DobotServer/SetHOMECmd` | `SetHOMECmd` | 机械臂回到零点 |
| `DobotServer/SetHOMEParams` | `SetHOMEParams` | 设置机械臂零点 |
| `DobotServer/SetQueuedCmdStartExec` | `SetQueuedCmdStartExec` | 开始执行队列命令 |
| `DobotServer/GetPose` | `GetPose` | 获取当前机械臂位姿 |
| `DobotServer/SetPTPCmd` | `SetPTPCmd` | 设置机械臂PTP运动 |
| `DobotServer/SetEndEffectorSuctionCup` | `SetEndEffectorSuctionCup` | 控制吸盘开关 |
| `DobotServer/ClearAllAlarmsState` | `ClearAllAlarmsState` | 清除机械臂报警状态 |

## 3. 关键函数及调用关系

### 3.1 Nav_Api.py

1. **Get_current_pose()**
   - 调用`rospy.wait_for_message('robot_pose', Pose)`同步获取机器人当前位姿。
   - 对外提供简洁API供其他模块调用。

2. **set_goal(x, y, z, x1, y1, z1, w1)**
   - 构造`MoveBaseActionGoal`消息并发布到`/move_base/goal`话题。
   - 消息中包含目标位置与四元数方向, 头信息统一使用`map`坐标系。

3. **cancel_goal()**
   - 发布空的`GoalID`到`/move_base/cancel`话题以取消所有导航目标。

4. **get_status()**
   - 使用`rospy.wait_for_message('/move_base/status', GoalStatusArray)`获取最新的导航状态。
   - 通过状态码映射字典将数值转为文本描述。

5. **navigate_to_position(position_data)**
   - 接受包含位置和姿态的字典, 调用`set_goal`发送导航目标。
   - 轮询`get_status()`直至到达或失败。

6. **set_xy_goal_tolerance() / set_yaw_goal_tolerance()**
   - 使用参数服务器调整TebLocalPlanner的xy和yaw容差, 返回布尔值表示设置是否成功。

### 3.2 robot_correct_position.py

- **correct_robot_position()**
  - 调用`Nav_Api.Get_current_pose()`获取当前位姿。
  - 构造`PoseWithCovarianceStamped`消息, 将位姿发送至`/initialpose`以触发AMCL重定位。

- **correct_robot_position_by_pose(pose)**
  - 接受字典格式的位姿, 重复上述发布流程, 用于将任意预定义点位设置为初始位姿。

### 3.3 robot_move_distance.py

- **SimpleForward.move_distance(distance)**
  - 按给定距离在`/cmd_vel`话题上发布线速度命令。
  - 借助TF监听器(`tf.TransformListener`)持续获取`/odom_combined`到`/base_footprint`的变换, 计算已移动距离并停止。

- **SimpleForward.get_position()**
  - 通过`lookupTransform`获取当前位姿, 返回`geometry_msgs/Point`对象。

### 3.4 QRcode_scan.py

- **get_province_info()**
  - 初始化临时ROS节点, 订阅`/usb_cam/image_raw`。
  - 使用`cv_bridge`将图像消息转换为OpenCV格式, 再用`pyzbar`解码二维码文本。
  - 从文本中提取省份名称, 最终以JSON字符串形式返回`{"count":数量,"provinces":[...省份...]}`。

- **parse_province_info(json_str)**
  - 解析上述JSON字符串, 提取并返回省份列表。

### 3.5 yaml_position_recorder.py

- 提供交互式命令行界面:
  - `'t'`: 从模板YAML加载点位名称并选择更新;
  - `'f'`: 刷新当前位置;
  - `'p'`: 打印当前已记录的点位;
  - `'s'`: 保存到YAML文件;
  - `'q'`: 退出不保存。
- 背景线程`run_position_correction`周期性调用`correct_robot_position()`保持定位。
- 保存格式示例:
  ```yaml
  point1:
    x: 1.0
    y: 2.0
    z: 0.0
    orientation_x: 0.0
    orientation_y: 0.0
    orientation_z: 0.0
    orientation_w: 1.0
    timestamp: 1711680000.0
  ```

### 3.6 yaml_box_delivery.py

- 整体流程:
  1. 读取`robot_position/positions.yaml`中的预定义点位(`origin`,`货架1.x`,`各省份`等)。
  2. 调用`correct_robot_position_by_pose()`将机器人重定位到`origin`。
  3. 使用导航API前往货架位置, 触发机械臂取盒子(`Dobot.dobot_take_box.main`)。
  4. 获取盒子上的二维码, 解析出目标省份。
  5. 通过`navigate_to_position`前往对应省份点位, 调用`Dobot.dobot_put_box.main`放置盒子。
  6. 过程间穿插`set_xy_goal_tolerance`/`set_yaw_goal_tolerance`调整导航精度和`correct_robot_position()`再次修正。

## 4. 机械臂(Dobot)相关

`Dobot/`目录下包含用于控制Dobot机械臂的脚本, 例如:

- `dobot_take_box.py`: 控制机械臂从货架取盒子。
- `dobot_put_box.py`: 控制机械臂在目标点放置盒子。

这些脚本通常通过串口或SDK与Dobot通信, 可能涉及串口配置、运动规划、吸盘控制等细节。AI在调用时需确保机械臂已连接并处于可用状态。

## 5. 工具库(utils)

`utils/`提供多个辅助函数:

- `getch`: 无回显地读取键盘按键, 用于交互式脚本。
- `read_yaml_file`: 读取YAML配置并返回字典。

## 6. 实现流程概述

典型的“取盒并配送”流程如下:

1. **定位与修正**
   - 使用`correct_robot_position_by_pose()`将机器人定位到预定义的`origin`点。
   - 后台线程持续调用`correct_robot_position()`保持定位准确。

2. **前往货架**
   - 调用`navigate_to_position()`前往货架点位, 并根据需要调整导航容差。

3. **机械臂取盒**
   - 到达后调用`Dobot.dobot_take_box.main()`执行取盒动作。

4. **识别目标省份**
   - 通过`QRcode_scan.get_province_info()`读取盒子上的二维码, 解析出目标省份。

5. **配送至省份点位**
   - 再次使用`navigate_to_position()`移动到省份点位。
   - 到达后使用`Dobot.dobot_put_box.main()`放置盒子。

6. **结束**
   - 若需要, 调用`cancel_goal()`取消残留的导航目标。

## 7. 依赖与环境

- **Python**: 主要使用Python 3, 部分兼容Python 2 (通过`from __future__ import print_function`和`reload(sys)`)。
- **ROS**: 依赖`rospy`、`geometry_msgs`、`move_base_msgs`、`actionlib_msgs`、`sensor_msgs`等标准消息包。
- **计算机视觉**: `opencv-python`, `pyzbar`, `cv_bridge`。
- **其他库**: `yaml`, `tf`、`threading`、`time`等。

## 8. 文件结构简述

- `Nav_Api.py`: 导航接口
- `robot_correct_position.py`: 位姿修正
- `robot_move_distance.py`: 定距离移动
- `QRcode_scan.py`: 二维码扫描
- `yaml_position_recorder.py`: 位置记录器
- `yaml_box_delivery.py`: 盒子配送主流程
- `Dobot/`: 机械臂控制脚本
- `utils/`: 工具函数
- `robot_position/`: 存储位姿YAML文件
- `test/`: 测试脚本 (依赖ROS环境运行)

## 9. 注意事项

1. 所有涉及ROS的话题或参数, 调用前需确保ROS Master已启动, 并且相关节点已运行。
2. `rospy`以及`cv_bridge`等依赖需要ROS环境才能安装和使用, 在纯Python环境下运行测试会报`ModuleNotFoundError`。
3. 机械臂控制脚本需要物理连接的Dobot硬件和正确的串口设置。
4. YAML文件路径通常基于当前脚本所在目录动态计算, 使用前需确认路径存在。

---

以上内容旨在帮助AI全面理解项目结构和各脚本之间的调用关系, 以便在未来的自动化操作中正确调用相关API、管理ROS话题、处理导航与机械臂任务。
