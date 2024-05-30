Here is the translation of the provided content from the "Calibration Tutorial for ORB-SLAM3 v1.0" document into Chinese:

---

# ORB-SLAM3 校准教程 v1.0

胡安·J·戈麦斯·罗德里格斯, 卡洛斯·坎波斯, 胡安·D·塔尔多斯

2021年12月22日

## 1 介绍

本文档包含了ORB-SLAM3 v1.0的视觉和视觉惯性校准的简要说明。在此版本中，我们引入了一种新的校准文件格式，其主要新特性包括：

- 通过一致的命名提高了可读性。
- 提供立体图像内部校正选项。
- 提供输入图像内部调整大小的选项。

可以在Examples目录中找到新校准格式的使用示例。虽然我们推荐使用新的文件格式，但为了用户的便利，我们保持与以前版本中使用的文件格式的兼容性，这些示例在Examples_old目录中。

## 2 参考系统和外参

外参校准包括定义不同传感器几何关系的一组参数。最复杂的情况是立体惯性配置，如图1所示，我们定义了以下参考系：

- 世界 (W)：定义一个固定的参考系统，其zW轴指向与重力矢量g相反的方向。平移和偏航由SLAM系统自由设置，并在初始化后保持固定。在纯视觉情况下，世界参考系设置为第一摄像机位置。
- 机体 (B)：这是可优化的参考系，并与IMU相连。我们假设陀螺仪和加速度计共享相同的参考系统。机体位置TWB和在W中表达的速度vB是可优化的变量。
- 摄像机 (C1和C2)：这些与摄像机的光学中心重合，zC沿光轴向前指向，yC向下指向，xC向右指向，都与图像方向u和v对齐。

图1：ORB-SLAM3立体惯性定义的参考系统。

摄像机和机体位置关系为：

\[TWC1 = TWBTBC1\]
\[TWC2 = TWBTBC1TC1C2\]

例如，TWC1 ∈ SE(3)是将摄像机一参考系中表达的点转换到世界参考系中的齐次变换：

\[xW = TWC1xC1\]

校准文件需要提供的外参为：

- 立体惯性：TBC1 和 TC1C2
- 单目惯性：TBC1
- 立体：仅TC1C2，ORB-SLAM3估计左摄像机的姿态 (B = C1)
- 单目：不需要参数，ORB-SLAM3估计摄像机姿态。

如果您的摄像机或数据集提供已校正的立体图像，校准文件只需指定基线，而不是完整的TC1C2变换。

所有的外参都可以使用校准软件如Kalibr [3]获取，如第4节所述。

## 3 内参

这些是仅取决于每个传感器本身的校准参数。这里我们区分惯性传感器和视觉传感器。

### 3.1 摄像机内参

根据摄像机设置，我们需要提供不同的校准参数。这些可以使用OpenCV或Kalibr [3]进行校准。在ORB-SLAM3中，我们区分三种摄像机类型：

- 针孔。需要提供的内参包括摄像机焦距和像素中心点（fx, fy, cx, cy），以及径向切向畸变模型[4]的参数，包括两或三个径向畸变参数（k1, k2, k3）和两个切向畸变系数（p1, p2）。
- KannalaBrandt8。Kannala-Brandt模型[2]适用于带有广角和鱼眼镜头的摄像机。需要提供的摄像机参数包括焦距和像素中心点（fx, fy, cx, cy）以及四个等距畸变模型系数（k1, k2, k3, k4）。
- 已校正。这种类型的摄像机用于提供校正后的立体图像的摄像机或数据集。在这种情况下，校准文件只需要提供校正图像的（fx, fy, cx, cy）和立体基线b（以米为单位）。

对于立体针孔摄像机，ORB-SLAM3内部使用OpenCV的stereorectify函数进行左图和右图的校正。为了避免分辨率和视野损失，不校正KannalaBrandt8类型的摄像机。

可以在目录Examples/Stereo-Inertial中找到这三种摄像机类型的校准文件示例，文件分别是Euroc.yaml（针孔）、TUM_VI_512.yaml（KannalaBrandt8）和Realsense_D435i.yaml（已校正）。

### 3.2 IMU内参

IMU读数（线性加速度ã和角速度ω̃）受到测量噪声（ηa,ηg）和偏差（ba,bg）的影响，如下所示：

\[ã =a+ ηa + ba\]
\[ω̃ =ω + ηg + bg\]

其中a和ω是机体参考系B中的真实加速度（未减去重力）和角速度。测量噪声假设服从中心正态分布，如下所示：

\[ηa ∼ N (0, σ^2_aI3)\]
\[ηg ∼ N (0, σ^2_gI3)\]

其中σa和σg是噪声密度，在IMU数据表中进行表征。需要在校准文件中提供它们，单位分别为m/s^1.5和rad/s^0.5，如清单1所示。

1 IMU.NoiseAcc: 0.0028 # m/s^1.5
2 IMU.NoiseGyro: 0.00016 # rad/s^0.5
3 IMU.Frequency: 200 # s^-1

清单1：IMU的噪声密度。数值来自TUM-VI数据集

当集成IMU测量并估计其协方差时，使用的噪声密度σa,f将取决于IMU采样频率f，必须在校准文件中提供。这由ORB-SLAM3内部管理，它计算

\[σa,f = σa/√f\]

关于偏差，假设它们根据布朗运动演变。给定两个连续时刻i和i+1，其特征为：

\[ba_{i+1} = ba_{i} + η_{a_{rw}}，其中η_{a_{rw}} ∼ N (0, σ^2_{a_{rw}}I3)\]
\[bg_{i+1} = bg_{i} + η_{g_{rw}}，其中η_{g_{rw}} ∼ N (0, σ^2_{g_{rw}}I3)\]

需要在校准文件中提供σa,rw和σg,rw，如清单2所示。

1 IMU.AccWalk: 0.00086 # m/s^2.5
2 IMU.GyroWalk: 0.000022 # rad/s^1.5

清单2：IMU偏差的随机游走标准差。数值来自TUM-VI数据集。

通常的做法是增加IMU制造商提供的随机游走标准差（例如乘以10），以考虑未建模的影响并提高IMU初始化收敛性。

## 4 使用Kalibr校准Realsense D435i的示例

这里我们展示了一个Realsense D435i设备在单目惯性配置下的视觉惯性校准示例。我们假设已经安装了realsense库（https://github.com/IntelRealSense/librealsense）。对于校准，我们将使用成熟的Kalibr开源软件[1, 3]，可以在https://github.com/ethz-asl/kalibr找到。我们将按以下步骤进行：

- 首先，为简便起见，我们将使用Kalibr CDE。只需从https://github.com/ethz-asl/kalibr/wiki/downloads下载。
- 作为校准图案，我们将采用建议的April标签。您可以在前述下载页面找到该图案的模板。需要打印它，并确保图案仅缩放而比例不变。更新此图案的配置.yaml文件。您可以在同一下载页面找到A0大小的模板。
- 要记录校准序列，可以使用realsense的SDK或我们提供的记录器，只需运行：

1 ./Examples/Calibration/recorder_realsense_D435i ./Examples/Calibration/recorder

清单3：运行视觉惯性记录器

确保记录器目录存在并包含空文件夹cam0和IMU。

- 使用Python脚本处理IMU数据并插值加速度

计测量，以与陀螺仪同步。

1 python3 ./Examples/Calibration/python_scripts/process_imu.py ./Examples/Calibration/recorder/

清单4：运行视觉惯性记录器

- 接下来，您需要使用Kalibr的rosbag创建器将此数据集转换为rosbag。从Kalibr CDE仓库运行以下命令：

1 ./kalibr_bagcreater --folder /path_to_ORB_SLAM3/Examples/Calibration/recorder/. --output-bag /path_to_ORB_SLAM3/Examples/Calibration/recorder.bag

清单5：运行视觉惯性记录器

### 4.1 视觉校准

此步骤解决相机内参以及立体传感器的相对相机变换。我们将按照以下步骤进行：

- 按照前述步骤，录制一组数据集，使用慢动作以避免图像模糊，并在良好照明下拍摄以减少曝光时间。也可以移动图案而不是摄像机，但要确保其不变形。示例校准序列可在https://youtu.be/R_K9-O4ool8找到。
- 使用尽可能大的图案，以便尽可能远地填满整个图像，简化摄像机对焦。此数据集没有最小尺寸，只需尝试从不同视角拍摄，确保所有像素多次看到图案。
- 您可以降低校准的帧速率，以加快Kalibr解决方案，但这不是必须的。
- 最后从Kalibr CDE仓库运行校准：

1 ./kalibr_calibrate_cameras --bag /path_to_ORB_SLAM3/Examples/Calibration/recorder.bag --topics /cam0/image_raw --models pinhole -radtan --target /path_to_ORB_SLAM3/Examples/Calibration/april_to_be_updated.yaml

清单6：运行视觉校准

对于我们的单目Realsense D435i，我们在表1中比较Kalibr和工厂校准。成功的视觉校准的典型重投影误差值是均值接近零像素（|μ| < 10−4）且标准差σ < 0.3。

### 4.2 惯性校准

校准视觉参数后，我们将继续进行惯性校准：

- 首先，录制一组新的数据集，进行快速运动，尝试沿所有轴激发加速度计和陀螺仪。尽量保持图案在图像中可见，以便Kalibr可以准确估计其相对姿态。图案应保持固定。适合的视觉惯性校准序列可在https://youtu.be/4XkivVLw5k4找到。
- 您需要提供IMU传感器的噪声密度和偏差随机游走。您可以从IMU数据表（Realsense D435i的BMI055）找到这些值，或使用长时间保持IMU静止时获取的数据进行估计。
- 最后从Kalibr CDE仓库运行校准：

1 ./kalibr_calibrate_imu_camera --bag /path_to_ORB_SLAM3/Examples/Calibration/recorder.bag --cam /path_to_ORB_SLAM3/Examples/Calibration/camera_calibration.yaml --imu /path_to_ORB_SLAM3/Examples/Calibration/imu_intrinsics.yaml --target /path_to_ORB_SLAM3/Examples/Calibration/april_to_be_updated.yaml

清单7：运行视觉校准

对于我们的单目Realsense D435i，我们在表2中比较Kalibr和工厂校准。

### 4.3 启动ORB-SLAM3

使用所有这些校准参数，您最终可以创建用于ORB-SLAM3的.yaml文件。最后，运行ORB-SLAM启动器：

1 ./Examples/Monocular-Inertial/mono_inertial_realsense_D435i Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/RealSense_D435i.yaml

清单8：运行实时单目惯性SLAM

## 5 新校准文件的参考

本节总结了ORB-SLAM3在其任何阶段所需的所有参数，包括内参和外参，ORB提取参数和可视化设置。所有这些配置参数都必须以yaml文件的形式传递给ORB-SLAM3。在这种类型的文件中，数据以<Key, value>对的形式存储，编码参数名称及其值。我们现在定义ORB-SLAM3接受的所有参数、它们的类型以及是否必须。

### 5.1 一般校准参数

这些是一般校准参数：

- File.version = "1.0" [必需]：指定正在使用的新校准文件格式。
- Camera.type（字符串）[必需]：指定使用的摄像机类型。必须取以下值之一：
    - PinHole：使用针孔摄像机时。
    - KannalaBrandt8：使用Kannala-Brandt校准模型的鱼眼摄像机时。
    - Rectified：使用立体校正的针孔摄像机时。
- Camera.height, Camera.width（整数）[必需]：输入图像的高度和宽度。
- Camera.newHeight, Camera.newWidth（整数）[可选]：如果定义，ORB-SLAM3会将输入图像调整到指定的新分辨率，并重新计算校准参数。
- Camera.fps（整数）[必需]：视频序列的每秒帧数。
- Camera.RGB（整数）[必需]：指定图像是BGR（0）还是RGB（1）。如果图像是灰度图像，则忽略。
- System.thFarPoints（浮点数）[可选]：如果定义，指定允许的最大深度（以米为单位）。超出此深度的点将被忽略。

### 5.2 摄像机内参

我们将Camera1定义为单目摄像机（在单目SLAM中）或左摄像机（在立体SLAM中）。其内参如下：

- Camera1.fx, Camera1.fy, Camera1.cx, Camera1.cy（浮点数）[必需]：对应于摄像机1的内参。

如果Camera.type设置为PinHole，您应指定：

- Camera1.k1, Camera1.k2（浮点数）[必需]：对应于径向畸变系数。
- Camera1.p1, Camera1.p2（浮点数）[必需]：对应于切向畸变系数。
- Camera1.k3（浮点数）[可选]：有时会使用第三个径向畸变参数。您可以用此参数指定。

如果Camera.type设置为KannalaBrandt8，您应指定：

- Camera1.k1, Camera1.k2, Camera1.k3, Camera1.k4（浮点数）[必需]：Kannala-Brandt畸变系数。

如果使用立体摄像机，还需指定Camera2（右摄像机）的参数，除非Camera.type设置为Rectified，在这种情况下，假定两台摄像机具有相同的参数。

### 5.3 立体参数

立体配置需要以下参数：

- Stereo.ThDepth（浮点数）[必需]：用于将点分类为近点或远点的立体基线数量。在立体SLAM算法的多个部分中，近点和远点的处理方式不同。

如果Camera.type设置为Rectified，您需要添加以下参数：

- Stereo.b（浮点数）[必需]：立体基线，以米为单位。

如果使用未校正的立体，您需要提供：

- Stereo.T_c1_c2（cv::Mat）[必需]：立体摄像机之间的相对姿态。

如果使用立体鱼眼摄像机（即Camera.type设置为KannalaBrandt8），您还需要指定两个图像的重叠区域：

- Camera1.overlappingBegin, Camera2.overlappingBegin（整数）[必需]：重叠区域的起始列。
- Camera1.overlappingEnd, Camera2.overlappingEnd（整数）[必需]：重叠区域的结束列。

### 5.4 惯性参数

使用惯性传感器时，用户必须定义以下参数：

- IMU.NoiseGyro（浮点数）[必需]：陀螺仪的测量噪声密度。
- IMU.NoiseAcc（浮点数）[必需]：加速度计的测量噪声密度。
- IMU.GyroWalk（浮点数）[必需]：陀螺仪的随机游走方差。
- IMU.AccWalk（浮点数）[必需]：加速度计的随机游走方差。
- IMU.Frequency（浮点数）[必需]：IMU频率。
- IMU.T_b_c1（cv::Mat）[必需]：IMU与摄像机1之间的相对姿态（即从摄像机1到IMU的变换）。
- IMU.InsertKFsWhenLost（整数）[可选]：指定当视觉跟踪丢失

但惯性跟踪仍在时，系统是否插入关键帧。

### 5.5 RGB-D 参数

RGB-D摄像机需要以下参数：

- RGBD.DepthMapFactor（浮点数）[必需]：将深度图转换为实际单位的因子。

### 5.6 ORB参数

这些参数与ORB特征提取有关：

- ORBextractor.nFeatures（整数）[必需]：每张图像要提取的特征数量。
- ORBextractor.scaleFactor（浮点数）[必需]：图像金字塔层之间的缩放因子。
- ORBextractor.nLevels（整数）[必需]：图像金字塔的层数。
- ORBextractor.iniThFAST（整数）[必需]：检测FAST角点的初始阈值。
- ORBextractor.minThFAST（整数）[必需]：如果初始阈值未检测到特征，系统会第二次尝试此阈值。

### 5.7 Atlas参数

这些参数定义是否从文件加载/保存地图：

- System.LoadAtlasFromFile（字符串）[可选]：要加载的地图所在文件路径。
- System.SaveAtlasToFile（字符串）[可选]：生成的地图保存的目标文件。

### 5.8 Viewer参数

这些参数与ORB-SLAM3用户界面有关：

- Viewer.KeyFrameSize（浮点数）[必需]：在地图查看器中绘制关键帧的大小。
- Viewer.KeyFrameLineWidth（浮点数）[必需]：关键帧绘图的线宽。
- Viewer.GraphLineWidth（浮点数）[必需]：共视图的线宽。
- Viewer.PointSize（浮点数）[必需]：地图点绘图的大小。
- Viewer.CameraSize（浮点数）[必需]：在地图查看器中绘制当前相机的大小。
- Viewer.CameraLineWidth（浮点数）[必需]：当前相机绘图的线宽。
- Viewer.ViewpointX, Viewer.ViewpointY, Viewer.ViewpointZ, Viewer.ViewpointF（浮点数）[必需]：地图查看器的起始视点。
- Viewer.imageViewScale（浮点数）[可选]：图像可视化的缩放因子（仅用于可视化，不用于SLAM管道）。

## 参考文献

[1] Paul Furgale, Joern Rehder, and Roland Siegwart. Unified temporal and spatial calibration for multi-sensor systems. In IROS, pages 1280–1286. IEEE, 2013.

[2] Juho Kannala and Sami S Brandt. A generic camera model and calibration method for conventional, wide-angle, and fish-eye lenses. IEEE Trans. Pattern Analysis and Machine Intelligence, 28(8):1335–1340, 2006.

[3] Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, and Roland Siegwart. Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. In Proc. IEEE Int. Conf. Robotics and Automation (ICRA), pages 4304–4311, 2016.

[4] Richard Szeliski. Computer vision: algorithms and applications. Springer Verlag, London, 2011.

---

This translation covers the provided text of the Calibration Tutorial for ORB-SLAM3 v1.0 document. If you need more sections of the document translated, please let me know.