# 📡 ROS 控制系统接口提示文档（For AI 编码）

本项目基于 ROS 开发，包含移动机器人导航、位姿修正、机械臂控制、视觉识别等模块。AI 写代码时请参考以下话题、服务和消息类型说明，确保正确调用。

---

## 🟩 话题（Topics）

| 话题名                  | 消息类型                                  | 功能说明                                             |
|-------------------------|--------------------------------------------|------------------------------------------------------|
| `/move_base/goal`       | `move_base_msgs/MoveBaseActionGoal`        | 设置机器人导航目标点（位置+姿态）                   |
| `/move_base/cancel`     | `actionlib_msgs/GoalID`                    | 取消当前导航任务                                     |
| `/move_base/status`     | `actionlib_msgs/GoalStatusArray`           | 获取导航状态列表（查看最后一项为当前状态）           |
| `/robot_pose`           | `geometry_msgs/Pose`                       | 获取机器人当前位姿（可用于同步阻塞等待）             |
| `/initialpose`          | `geometry_msgs/PoseWithCovarianceStamped`  | 发布初始位姿到 AMCL 系统以完成定位修正               |
| `/cmd_vel`              | `geometry_msgs/Twist`                      | 控制机器人底盘线速度（前进/后退/转弯）               |
| `/usb_cam/image_raw`    | `sensor_msgs/Image`                        | 摄像头图像流输入，供二维码或OCR识别模块使用          |

---

## 🟦 服务（Services）

| 服务名                                  | 服务类型（Srv）                         | 功能说明                                            |
|-----------------------------------------|------------------------------------------|-----------------------------------------------------|
| `/DobotServer/SetHOMECmd`               | `dobot/SetHOMECmd`                       | 控制机械臂返回预设的零点位置                        |
| `/DobotServer/SetHOMEParams`            | `dobot/SetHOMEParams`                    | 设置机械臂零点参数                                  |
| `/DobotServer/SetQueuedCmdStartExec`    | `dobot/SetQueuedCmdStartExec`            | 启动机械臂队列执行指令                             |
| `/DobotServer/GetPose`                  | `dobot/GetPose`                          | 获取机械臂当前末端位置                              |
| `/DobotServer/SetPTPCmd`                | `dobot/SetPTPCmd`                        | 控制机械臂点到点运动至指定坐标                      |
| `/DobotServer/SetEndEffectorSuctionCup` | `dobot/SetEndEffectorSuctionCup`         | 控制末端吸盘的启用与关闭（吸附/释放）              |
| `/DobotServer/ClearAllAlarmsState`      | `dobot/ClearAllAlarmsState`              | 清除机械臂的所有报警状态                           |

---

## 🟨 坐标系与 TF

| 源帧              | 目标帧            | 用途说明                                  |
|-------------------|-------------------|-------------------------------------------|
| `/odom_combined`  | `/base_footprint` | 通过 TF 获取机器人当前相对位移与旋转信息  |

---

## ⚙️ 可配置参数（rosparam）

| 参数路径                                           | 类型     | 功能说明                                  |
|----------------------------------------------------|----------|-------------------------------------------|
| `/move_base/TebLocalPlannerROS/xy_goal_tolerance`  | `float`  | 导航目标点 xy 坐标容差（单位：米）         |
| `/move_base/TebLocalPlannerROS/yaw_goal_tolerance` | `float`  | 导航目标点角度容差 yaw（单位：弧度）       |

---

## 💡 使用建议（For AI）

- 发布目标请使用 `/move_base/goal`，取消目标使用 `/move_base/cancel`。
- 获取机器人当前位姿优先用 `/robot_pose`（阻塞方式：`wait_for_message`）。
- 吸盘控制需调用服务 `/DobotServer/SetEndEffectorSuctionCup`。
- 使用 `/initialpose` 发布初始定位数据时，附带协方差。
- 控制底盘直线前进时使用 `/cmd_vel`，位移距离通过 TF 动态估算。

AI 在写代码时，请**严格根据以上接口设计交互逻辑**，避免虚构不存在的话题或服务。
