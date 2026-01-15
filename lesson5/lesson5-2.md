# Lesson 5-2: TurtleBot3 摄像头与点云传感器

## 学习目标

- 在 Gazebo 仿真中为 TurtleBot3 加载摄像头插件
- 理解 `image_transport` 图像传输机制
- 熟悉 `sensor_msgs/Image` 和 `sensor_msgs/PointCloud2` 消息类型
- 了解点云可视化与常见滤波思路

## 前置条件

- 完成 Lesson 4 和 Lesson 5 的学习
- TurtleBot3 仿真环境已安装
- RViz2 可正常使用

---

## 第一部分：TurtleBot3 摄像头仿真

### 1.1 选择带摄像头的模型

TurtleBot3 有三种模型，其中 `waffle` 和 `waffle_pi` 自带摄像头：

| 模型 | 摄像头 | 激光雷达 | 深度相机 |
|------|--------|----------|----------|
| burger | ❌ | ✅ LDS-01 | ❌ |
| waffle | ✅ Intel RealSense R200 | ✅ LDS-01 | ✅ |
| waffle_pi | ✅ Raspberry Pi Camera | ✅ LDS-01 | ❌ |

### 1.2 启动带摄像头的仿真

```bash
# 设置模型为 waffle（带深度相机）
export TURTLEBOT3_MODEL=waffle

# 启动 Gazebo 仿真
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 1.3 查看摄像头相关话题

```bash
# 列出所有图像相关话题
ros2 topic list | grep -E "(image|camera|depth)"
```

典型输出：
```
/camera/camera_info
/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/points
/camera/image_raw
```

### 1.4 话题说明

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/camera/image_raw` | sensor_msgs/Image | RGB 彩色图像 |
| `/camera/camera_info` | sensor_msgs/CameraInfo | 相机内参 |
| `/camera/depth/image_raw` | sensor_msgs/Image | 深度图像 |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | 3D 点云数据 |

---

## 第二部分：image_transport 图像传输

### 2.1 什么是 image_transport

`image_transport` 是 ROS 中专门用于图像传输的包，提供：

- **透明压缩**：自动处理图像压缩/解压
- **多种传输格式**：raw、compressed、theora 等
- **带宽优化**：减少网络传输负载

### 2.2 传输格式对比

| 格式 | 话题后缀 | 特点 |
|------|----------|------|
| raw | 无 | 原始图像，无压缩，带宽大 |
| compressed | /compressed | JPEG/PNG 压缩，带宽小 |
| compressedDepth | /compressedDepth | 深度图专用压缩 |
| theora | /theora | 视频流压缩 |

### 2.3 查看可用的传输格式

```bash
# 查看某个图像话题的传输格式
ros2 run image_transport list_transports
```

### 2.4 使用 compressed 格式订阅

```bash
# 安装 image_transport 插件
sudo apt install ros-humble-image-transport-plugins

# 订阅压缩图像
ros2 topic echo /camera/image_raw/compressed
```

### 2.5 图像话题结构

使用 `image_transport` 发布图像时，会自动创建多个话题：

```
/camera/image_raw              # 原始图像
/camera/image_raw/compressed   # JPEG 压缩
/camera/image_raw/theora       # Theora 视频流
```

---

## 第三部分：sensor_msgs/Image 消息详解

### 3.1 消息结构

```bash
ros2 interface show sensor_msgs/msg/Image
```

```yaml
# 消息头
std_msgs/Header header
  builtin_interfaces/Time stamp    # 时间戳
  string frame_id                  # 坐标系 ID

# 图像尺寸
uint32 height                      # 图像高度（像素）
uint32 width                       # 图像宽度（像素）

# 编码格式
string encoding                    # 像素编码格式

# 数据排列
uint8 is_bigendian                 # 大端序标志
uint32 step                        # 每行字节数（width * 通道数 * 字节深度）

# 图像数据
uint8[] data                       # 原始像素数据
```

### 3.2 常见编码格式

| encoding | 说明 | 通道数 | 字节/像素 |
|----------|------|--------|-----------|
| `rgb8` | RGB 彩色 | 3 | 3 |
| `bgr8` | BGR 彩色（OpenCV 默认） | 3 | 3 |
| `rgba8` | RGBA 带透明度 | 4 | 4 |
| `mono8` | 8位灰度 | 1 | 1 |
| `mono16` | 16位灰度 | 1 | 2 |
| `32FC1` | 32位浮点深度图 | 1 | 4 |
| `16UC1` | 16位无符号深度图 | 1 | 2 |

### 3.3 查看图像信息

```bash
# 查看图像话题详情
ros2 topic info /camera/image_raw -v

# 查看单帧图像数据（会很长）
ros2 topic echo /camera/image_raw --once
```

### 3.4 使用 RViz2 可视化图像

```bash
# 启动 RViz2
rviz2
```

添加 Image 显示：
1. 点击 "Add" 按钮
2. 选择 "By topic" → `/camera/image_raw` → "Image"
3. 图像将显示在 RViz2 中

---

## 第四部分：sensor_msgs/PointCloud2 消息详解

### 4.1 消息结构

```bash
ros2 interface show sensor_msgs/msg/PointCloud2
```

```yaml
# 消息头
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id

# 点云尺寸
uint32 height                      # 高度（有序点云）或 1（无序）
uint32 width                       # 宽度或点数

# 字段描述
sensor_msgs/PointField[] fields    # 每个点的数据字段
  string name                      # 字段名（x, y, z, rgb, intensity...）
  uint32 offset                    # 字段在点数据中的偏移
  uint8 datatype                   # 数据类型
  uint32 count                     # 元素数量

# 数据属性
bool is_bigendian
uint32 point_step                  # 每个点的字节数
uint32 row_step                    # 每行的字节数

# 点云数据
uint8[] data                       # 原始点云数据

bool is_dense                      # 是否包含无效点（NaN）
```

### 4.2 常见点云字段

| 字段名 | 类型 | 说明 |
|--------|------|------|
| x, y, z | FLOAT32 | 3D 坐标 |
| rgb | FLOAT32 | 打包的 RGB 颜色 |
| rgba | UINT32 | RGBA 颜色 |
| intensity | FLOAT32 | 反射强度 |
| ring | UINT16 | 激光雷达环号 |
| normal_x/y/z | FLOAT32 | 法向量 |

### 4.3 有序 vs 无序点云

| 类型 | height | width | 特点 |
|------|--------|-------|------|
| 有序点云 | >1 | >1 | 来自深度相机，保留像素结构 |
| 无序点云 | 1 | 点数 | 来自激光雷达，无结构 |

### 4.4 查看点云信息

```bash
# 查看点云话题
ros2 topic info /camera/depth/points -v

# 查看点云频率
ros2 topic hz /camera/depth/points

# 查看点云字段
ros2 topic echo /camera/depth/points --field fields
```

---

## 第五部分：RViz2 点云可视化

### 5.1 启动 RViz2 并添加点云

```bash
rviz2
```

配置步骤：
1. 设置 Fixed Frame 为 `odom` 或 `base_link`
2. 点击 "Add" → "By topic" → `/camera/depth/points` → "PointCloud2"
3. 调整显示参数

### 5.2 点云显示参数

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| Style | 渲染方式 | Points / Flat Squares |
| Size | 点大小 | 0.01 - 0.05 |
| Color Transformer | 着色方式 | RGB8 / Intensity / AxisColor |
| Decay Time | 历史点保留时间 | 0（只显示最新） |

### 5.3 着色方式说明

- **RGB8**：使用点云自带的颜色
- **Intensity**：按反射强度着色
- **AxisColor**：按 X/Y/Z 轴值着色（常用于查看高度）
- **FlatColor**：单一颜色

---

## 第六部分：点云滤波思路（理论）

### 6.1 为什么需要滤波

原始点云数据通常存在：
- **噪声点**：传感器误差产生的离群点
- **数据量大**：影响处理速度
- **无效点**：NaN 或超出范围的点
- **遮挡/反射**：玻璃、镜面等产生的错误点

### 6.2 常见滤波方法

#### 1. 直通滤波（PassThrough Filter）

**原理**：保留指定坐标范围内的点

**应用场景**：
- 去除地面点
- 限制感兴趣区域
- 去除过远/过近的点

```
示例：只保留 z 轴 0.5m ~ 2.0m 范围内的点
```

#### 2. 体素滤波（Voxel Grid Filter）

**原理**：将空间划分为小立方体（体素），每个体素内的点用一个代表点替代

**应用场景**：
- 降采样，减少点数
- 保持点云整体形状
- 加速后续处理

```
示例：体素大小 0.01m，点数可减少 90%+
```

#### 3. 统计滤波（Statistical Outlier Removal）

**原理**：计算每个点到邻近点的平均距离，去除距离异常的点

**应用场景**：
- 去除离群噪声点
- 平滑点云边缘

```
参数：邻近点数 K=50，标准差阈值 σ=1.0
```

#### 4. 半径滤波（Radius Outlier Removal）

**原理**：检查每个点在指定半径内的邻近点数，点数不足则删除

**应用场景**：
- 去除稀疏噪声
- 保留密集区域

```
参数：搜索半径 r=0.05m，最小邻近点数 n=5
```

### 6.3 滤波流程建议

```
原始点云
    ↓
1. 直通滤波（去除无效范围）
    ↓
2. 体素滤波（降采样）
    ↓
3. 统计滤波（去除噪声）
    ↓
处理后点云
```

### 6.4 PCL 库简介

**PCL（Point Cloud Library）** 是点云处理的标准库：

- 官网：https://pointclouds.org/
- ROS 2 包：`pcl_ros`、`pcl_conversions`

常用功能：
- 滤波（Filtering）
- 分割（Segmentation）
- 配准（Registration）
- 特征提取（Feature Extraction）
- 表面重建（Surface Reconstruction）

---

## 第七部分：实践命令汇总

### 启动仿真

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 查看话题

```bash
# 图像话题
ros2 topic list | grep image

# 点云话题
ros2 topic list | grep points

# 查看消息类型
ros2 topic info /camera/image_raw
ros2 topic info /camera/depth/points
```

### 可视化

```bash
# 使用 rqt_image_view 查看图像
ros2 run rqt_image_view rqt_image_view

# 使用 RViz2 查看点云
rviz2
```
rqt_image_view 中查看图像
点击左上角的下拉框（切换到显示为空）
点击旁边的刷新按钮（🔄）刷新话题列表。
选择 /camera/image_raw
图像就会显示出来


在 RViz2 中可视化激光雷达：

打开 rviz2
设置 Fixed Frame 为 odom 或 base_link
点击 Add → By topic → /scan → LaserScan
你会看到 2D 激光扫描线



### 录制数据

```bash
# 录制图像和点云
ros2 bag record /camera/image_raw /camera/depth/points -o sensor_data

# 回放
ros2 bag play sensor_data
```

---

## 总结

本课程学习了：

- ✅ TurtleBot3 waffle 模型自带摄像头和深度相机
- ✅ `image_transport` 提供多种图像传输格式
- ✅ `sensor_msgs/Image` 包含图像尺寸、编码、原始数据
- ✅ `sensor_msgs/PointCloud2` 包含点云字段定义和 3D 数据
- ✅ RViz2 可视化图像和点云
- ✅ 点云滤波：直通、体素、统计、半径滤波

---

## 参考资料

- [sensor_msgs 文档](https://docs.ros2.org/latest/api/sensor_msgs/)
- [image_transport Wiki](http://wiki.ros.org/image_transport)
- [PCL 官方文档](https://pcl.readthedocs.io/)
- [TurtleBot3 传感器规格](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
