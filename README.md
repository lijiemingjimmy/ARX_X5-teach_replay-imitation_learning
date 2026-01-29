# ARX X5 项目概述与实现记录（可放 GitHub）

> 适用对象：ARX X5 单臂（AC one，左臂），ROS2 Jazzy（Ubuntu 24.04）
>
> 更新时间：2026-01-29

---

## 1. 项目概览

本仓库记录了 ARX X5 机械臂的启动流程与后续功能实现（键盘遥控、teach/replay、仿真训练）。核心目标：**把官方 SDK / ROS2 驱动跑通，并在其上做安全、可复用的上层控制与数据采集。**

项目原始仓库（官方）：

- `https://github.com/ARXroboticsX/ARX_X5`

相关参考：

- `https://github.com/real-stanford/arx5-sdk`
- `https://github.com/ARISE-Initiative/robomimic`

---

## 2. 设备信息（参考值）

> 以下参数来自公开视频/论文/复现文档，存在轻微差异，仅用于量级参考。

![ARX X5 机械臂](docs/images/arx_x5_robot.jpg)
*ARX X5 机械臂实物图*

- 自由度：6 DoF
- 质量：约 3.3 kg（含夹爪）
- 工作半径：约 620 mm（ARX5），ARX5a 约 510 mm
- 负载：约 1.5–2 kg（不同资料略有差异）

---

## 3. 仓库目录结构（官方仓库）

- `ROS/X5_ws`：ROS1 工作空间
- `ROS2/X5_ws/src`：ROS2 工作空间源码
- `ARX_CAN`：CAN 总线通信相关代码
- `arx_joy/src/arx_joy`：手柄遥控 ROS 包
- `py/arx_x5_python`：Python 侧接口
- `ARX_VR_SDK`：VR 控制 SDK
- `00-readme` / `00-sh`：安装与启动脚本

> 当前使用的是单臂版本（AC one），VR/手柄控制无法使用。

---

## 4. 启动流程（ROS2 Jazzy）

### 4.1 环境准备

![环境配置](docs/images/environment_setup.png)
*Ubuntu 24.04 + ROS2 Jazzy 环境配置*

- 操作系统：Ubuntu 24.04（建议 Desktop 版）
- ROS2：Jazzy
- 机械臂型号：AC one
- 启动脚本：以 `v2` 标识

> 由于 Apple Silicon 上的虚拟机无法运行官方脚本，最终在 x86_64 设备上安装 Ubuntu 24.04（双系统）。

---

### 4.2 CAN 设备绑定

![CAN 设备连接](docs/images/can_device.jpg)
*CAN USB 设备连接示意图*

步骤：

1. 运行 `search.sh` 查找 serial
2. 修改 `arx_can.rules`
3. 运行 `set.sh` 生效
4. 启动对应 `arx_can*.sh`

注意：

- 每次只能插 **一个** CAN USB（逐个绑定）
- AC one 左臂使用 `can1`

---

### 4.3 启动 CAN

```bash
./arx_can1.sh
```

看到 `CAN1` 正常后继续下一步。

---

### 4.4 启动 ROS2 控制节点

```bash
cd ARX_X5/ROS2/X5_ws
source install/setup.bash

# X5-2023
ros2 launch arx_x5_controller open_single_arm.launch.py
```

> 注意：只允许 **一个控制终端** 运行，否则会抖动或抢占控制权。

---

### 4.5 rqt 测试

![rqt 控制界面](docs/images/rqt_interface.png)
*使用 rqt 进行关节控制测试*

```bash
cd ARX_X5/ROS2/X5_ws
source install/setup.bash
rqt
```

验证链路时，建议角度不要超过 ±0.1 rad（夹爪除外）。

---

### 4.6 查看反馈

```bash
source install/setup.bash
ros2 topic echo /arm_status
```

字段解释：

- `end_pos`：末端执行器位置/位姿
- `joint_pos`：6 轴关节角
- `gripper`：夹爪开合状态

---

## 5. Keyboard Control（关节空间）

### 5.1 原理

持续订阅 `/arm_status` 获取当前关节角 → 按键触发小增量 → 发布 `/arm_cmd`。

### 5.2 创建 ROS2 包

```bash
cd ~/Downloads/ARX_X5-main/ROS2/X5_ws
source install/setup.bash
cd src
ros2 pkg create --build-type ament_python arx_keyboard_teleop
```

### 5.3 主程序

文件：

- `ROS2/X5_ws/src/arx_keyboard_teleop/arx_keyboard_teleop/teleop_joint.py`

### 5.4 注册入口

编辑 `setup.py`：

```python
entry_points={
    'console_scripts': [
        'teleop_joint = arx_keyboard_teleop.teleop_joint:main',
    ],
},
```

### 5.5 编译运行

```bash
cd ~/Downloads/ARX_X5-main/ROS2/X5_ws
colcon build --packages-select arx_keyboard_teleop
source install/setup.bash
ros2 run arx_keyboard_teleop teleop_joint
```

---

### 5.6 安全机制（强烈建议）

![键盘遥控演示](docs/images/keyboard_teleop.gif)
*键盘关节空间遥控演示*

- **Deadman 键**：不按不动，松手急停（推荐 `Shift` 或 `Space`）
- **速度/加速度上限**：避免抖动/冲击
  - 关节速度上限：5°/s（约 0.087 rad/s）
  - 加速度上限：20°/s²（约 0.35 rad/s²）
  - 发布频率：50 Hz
- **Watchdog**：0.2s 内无按键更新 → 自动停
- **软限位 + 姿态互锁**：关键关节动作依赖其他关节角度

示例互锁逻辑：

```python
allow_move(joint_id, q):
    if joint_id in {J5, J6}:
        return (q3 in safe_range3) and (q4 in safe_range4)
    return True
```

---

## 6. Keyboard Control（笛卡尔空间）

### 6.1 原理

订阅 `/arm_status` 取 `end_pos` 作为起点，按键输出末端速度，积分得到目标位姿，发布 `/arm_cmd`（`mode=4`）。

### 6.2 创建 ROS2 包

```bash
cd ~/Downloads/ARX_X5-main/ROS2/X5_ws
source install/setup.bash
cd src
ros2 pkg create --build-type ament_python arx_keyboard_cart_teleop
```

### 6.3 主程序

- `ROS2/X5_ws/src/arx_keyboard_cart_teleop/arx_keyboard_cart_teleop/teleop_cart.py`

### 6.4 注册入口

```python
entry_points={
    'console_scripts': [
        'teleop_cart = arx_keyboard_cart_teleop.teleop_cart:main',
    ],
},
```

### 6.5 编译运行

```bash
cd ~/Downloads/ARX_X5-main/ROS2/X5_ws
colcon build --packages-select arx_keyboard_cart_teleop
source install/setup.bash

# 先启动控制器，然后再运行
ros2 run arx_keyboard_cart_teleop teleop_cart
```

### 6.6 键位设计（示例）

![笛卡尔空间遥控](docs/images/cartesian_teleop.gif)
*键盘笛卡尔空间遥控演示*

- Deadman：`Shift`（按住才能动）
- 平移：
  - `W/S`：+X / -X
  - `A/D`：+Y / -Y
  - `R/F`：+Z / -Z
- 旋转（Roll/Pitch/Yaw）：
  - `I/K`：+Roll / -Roll
  - `J/L`：+Pitch / -Pitch
  - `U/O`：+Yaw / -Yaw
- 速度调节：
  - `Z/X`：平移速度减/增
  - `C/V`：旋转速度减/增
- 急停：`ESC`

---

## 7. 关节限位（参考）

> 实际限位应以官方参数为准。

- J1: `[-1.57,  1.57]`
- J2: `[-0.1,   3.6 ]`
- J3: `[-0.1,   3.0 ]`
- J4: `[-1.29,  1.29]`
- J5: `[-1.48,  1.48]`
- J6: `[-1.74,  1.74]`
- J7（夹爪）: `[-3.4, 0.1]`

---

## 8. Teach / Replay（SDK）

参考：`https://github.com/real-stanford/arx5-sdk`

### 8.1 安装 Miniforge（x86_64）

```bash
uname -m
cd ~
curl -L -o Miniforge3.sh https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash Miniforge3.sh -b -p "$HOME/miniforge3"
echo 'export PATH="$HOME/miniforge3/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

### 8.2 创建环境 & 编译 SDK

```bash
cd ~/arx5-sdk
mamba env create -f conda_environments/py310_environment.yaml
conda activate arx-py310

mkdir -p build && cd build
cmake ..
make -j
```

### 8.3 测试

```bash
cd ~/arx5-sdk/python
conda activate arx-py310
python examples/test_joint_control.py X5 can1
```

### 8.4 Teach / Replay

![Teach Replay 流程](docs/images/teach_replay_workflow.png)
*Teach & Replay 工作流程示意图*

```bash
python examples/teach_replay.py -h
```

### 8.5 自定义版本（teach_replay_new.py）

![数据采集过程](docs/images/data_collection.jpg)
*使用 Teach/Replay 进行数据采集*

目标：

- Teach 之前可指定新的零点
- Replay 后回到新零点
- 从新零点返回原点
- 支持多轮 replay

---

## 9. Robomimic 仿真训练

参考：`https://github.com/ARISE-Initiative/robomimic`

### 9.1 安装

```bash
conda create -n robomimic_venv python=3.8.0 -y
conda activate robomimic_venv
conda install pytorch==2.0.0 torchvision==0.15.1 -c pytorch -y

git clone https://github.com/ARISE-Initiative/robomimic.git
cd robomimic
pip install -e .
```

### 9.2 Debug

```bash
python examples/train_bc_rnn.py --debug
```

### 9.3 下载数据集

```bash
python robomimic/scripts/download_datasets.py --tasks lift --dataset_types ph
python robomimic/scripts/get_dataset_info.py --dataset datasets/lift/ph/low_dim_v141.hdf5
```

### 9.4 训练

![训练过程](docs/images/training_process.png)
*Robomimic 训练过程可视化*

```bash
python robomimic/scripts/train.py \
  --config robomimic/exps/templates/bc.json \
  --dataset datasets/lift/ph/low_dim_v141.hdf5 \
  --debug
```

### 9.5 可视化

![TensorBoard 监控](docs/images/tensorboard.png)
*使用 TensorBoard 监控训练指标*

```bash
tensorboard --logdir bc_trained_models/test --bind_all
```

---

## 10. 重要注意事项

- **不要同时运行 rqt publisher 和键盘节点发布 `/arm_cmd`**
- **一切控制操作先小幅度测试**（特别是首次启动）
- **必须有 Deadman**，否则误触风险极大
- **务必加速度/速度上限**，否则会抖动甚至损坏设备
- **建议加互锁逻辑**，防止在危险姿态下误动末端

---

## 11. 待补充（建议）

- 实际相机/机械臂实验图片（请换成仓库内相对路径）
- 实测的 ARX X5 真实关节限位参数
- 键盘遥控代码注释与安全参数配置文件
- 数据采集流程（动作 + 视觉 + 对齐策略）

---

## 12. 许可证与致谢

- ARX 官方仓库与 SDK 所有权归原作者
- 参考资料与第三方项目请遵循原仓库许可协议