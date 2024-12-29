

# 装甲板识别，测距，投影&机器人动态追随


特别鸣谢同济大学的分享，改编于同济大学的装甲板开源，详情请见https://www.bilibili.com/video/BV1qkxceWEh4/?spm_id_from=333.337.top_right_bar_window_history.content.click
本代码已实现图像预处理-->灯条识别-->装甲板位姿解算（pnp）-->结果可视化
（图像预处理：读取图片-->转换成灰度图-->进行二值化-->获取轮廓点-->获取旋转矩形）

**图像预处理**

**1.订阅**：订阅相机发布的图像话题（例如：`/color/image_raw`）。

**2.预处理**：将接收到的ROS图像消息转换为OpenCV的`cv::Mat`格式，并进行灰度化、二值化等预处理操作。

**灯条识别**

**1.识别**：在预处理后的图像中检测灯条的轮廓，并根据几何特征（如长度、宽度、角度等）筛选出可能的灯条。

**2.颜色判断**：根据灯条区域的颜色信息（如红色或蓝色）判断灯条的颜色。

**装甲板位姿解算（PnP）**

**1.装甲板构建**：根据识别到的灯条甲板的四个角的点，构建装甲板的几何形状。

**2.位姿解算**：使用PnP算法（`cv::solvePnP`）计算装甲板在相机坐标系下的位姿

（旋转向量`rvec`和平移向量`tvec`）。

`cv::solvePnP`函数会返回两个结果：
`rvec`：一个旋转向量，表示装甲板在现实世界中的方向。
`tvec`：一个平移向量，表示装甲板在现实世界中的位置。

- **欧拉角计算**：从旋转向量`rvec`中提取欧拉角，以获得装甲板的姿态信息。
- 
**结果可视化**

添加了一个图像发布者`image_pub_`，并在`imageCallback`函数中使用`cv_bridge`将处理后的图像转换为ROS消息，并发布到`/camera/image_processed`，在 `imageCallback` 函数中，使用 OpenCV 的 `cv::namedWindow`、`cv::resizeWindow`、`cv::imshow` 和 `cv::waitKey` 函数创建一个可调整大小的窗口，并显示处理后的图像。`cv::waitKey(1)` 允许 OpenCV 窗口更新显示的内容。

**结果可视化**

添加了一个图像发布者`image_pub_`，并在`imageCallback`函数中使用`cv_bridge`将处理后的图像转换为ROS消息，并发布到`/camera/image_processed`，在 `imageCallback` 函数中，使用 OpenCV 的 `cv::namedWindow`、`cv::resizeWindow`、`cv::imshow` 和 `cv::waitKey` 函数创建一个可调整大小的窗口，并显示处理后的图像。`cv::waitKey(1)` 允许 OpenCV 窗口更新显示的内容。

**pnp算法实现逻辑**
1. **输入**：

  `object_points`：装甲板在物体坐标系下的四个角点的三维坐标，

  这些坐标就是这个模型上四个角点在模型自身空间中的位置。

  `img_points`：装甲板在图像平面上的四个角点的二维坐标。

  这是装甲板在相机拍摄的图像平面上的四个角点的二维坐标。

  就好像你在照片上看到装甲板的四个角，这些坐标就是它们在照片上的位置。

  `camera_matrix`：相机的内参矩阵。

  它描述了相机的内部参数，比如焦距、图像中心等。

  可以把它想象成相机的“眼睛”的特性，告诉我们相机是如何看待世界的。

  `distort_coeffs`：相机的畸变系数。

  这是相机的畸变系数，用于校正图像的畸变。因为相机镜头可能会使图像产生一些扭曲，这些系数可以帮助我们把图像变回原来的样子。

2. **计算**：使用`cv::solvePnP`函数求解PnP问题，得到旋转向量`rvec`和平移向量`tvec`。

3. **输出**：`rvec`：装甲板在相机坐标系下的旋转向量。

             `tvec`：装甲板在相机坐标系下的平移向量。

`rvec`：一个旋转向量，表示装甲板在现实世界中的方向。
`tvec`：一个平移向量，表示装甲板在现实世界中的位置。


'<cv::Mat rvec, tvec;
cv::solvePnP(object_points, img_points, camera_matrix, distort_coeffs, rvec, tvec);>'

![截图 2024-12-29 16-12-06](https://github.com/user-attachments/assets/f05bca7e-c76e-4bab-8f2c-14c5aaee7a2f)


### 使用方法

'<cd ~/Downloads/armor_ws>'  根据你自己下载的路径更改


'<colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON>'


'<source install/setup.bash>'



'<ros2 run armor_detector armor_detector>'
