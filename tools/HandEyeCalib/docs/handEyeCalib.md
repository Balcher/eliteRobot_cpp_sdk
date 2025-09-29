# 手眼标定理论

机械臂手眼标定的**主要目的**是==将相机对目标的位姿识别结果转换为机械臂坐标系下的位姿坐标==，从而使得机械臂能够借助视觉信息执行相应的任务。

根据相机的位置可以分为眼在手上以及眼在手外，原理推导有些许差异，但是求解方法是一样的。

首先计算出相机坐标系与机械臂基坐标系之间的位姿关系，只有获得这两个坐标系之间的位姿变换关系，才能将相机坐标系系的夹爪目标位姿变换到机械臂基坐标系下，进而执行机械臂抓取操作。

手眼标定的过程其实就是在标定相机相对于机械臂基坐标系的位置关系，这样从相机看到的物体位姿（相机坐标系下的物体位姿）转换到机械臂基坐标系（物体相对于机械臂基坐标系的转换矩阵），因为末端执行器到机械臂基坐标系的位置关系是知道的（通过DH坐标系），然后给机械臂坐标值（机械臂基坐标系下的）机械臂通过SDK能够自主到达。

# 眼在手外

![image-20250124130433941](.\image\image-20250124130433941.png)

相机到机械臂基坐标系的位置关系为$X$ ，手眼标定就是在求这个$X$。

# 眼在手上

![image-20250124130824456](.\image\image-20250124130824456.png)

眼在手上相当于是标定板不动，机械臂带着相机动，此时求的是相机到末端的位姿。

# OpenCV 手眼标定的API

OpenCV中的`cv::calibrateHandEye` 是进行**手眼标定**的函数。

这个标定的是相机到夹爪的变换矩阵。

```cpp
CV_EXPORTS_W void calibrateHandEye( InputArrayOfArrays R_gripper2base, 
                                    InputArrayOfArrays t_gripper2base, 
                                    InputArrayOfArrays R_target2cam, 
                                    InputArrayOfArrays t_target2cam,
                                    OutputArray R_cam2gripper, 
                                    OutputArray t_cam2gripper, 
                                    HandEyeCalibrationMethod method=CALIB_HAND_EYE_TSAI                                                                                                        
                                    );
```

- `R_gripper2base` ：【in】从齐次矩阵中提取的旋转部分，用于将一个点从夹爪坐标系转换到机器人基坐标系。
- `t_gripper2base` ：【in】从齐次矩阵中提取的平移部分，用于将一个点从夹爪坐标系转换到机器人基坐标系。
- `R_target2cam` ：【in】从齐次矩阵中提取的旋转部分，用于将一个点从目标坐标系（棋盘格）转换到相机坐标系。
- `t_target2cam` ：【in】从齐次矩阵中提取的平移部分，用于将一个点从目标坐标系（棋盘格）转换到相机坐标系。
- `R_cam2gripper` ：【out】估计的 `(3x3)` 旋转部分，表示将一个点从相机坐标系转换到夹爪坐标系。
- `t_cam2gripper` ：【out】估计的 `(3x1)` 平移部分，表示将一个点从相机坐标系转换到夹爪坐标系。
- `method` ：手眼标定方法之一

**功能：**

该函数通过多种方法执行手眼标定。一种方法是先估计旋转矩阵，再估计平移向量（可分离解），已经实现的手眼标定方法有：

- **R. Tsai, R. Lenz** 《一种全自主且高效的三维机器人手眼标定技术》 \cite Tsai89
- **F. Park, B. Martin** 《机器人传感器标定：解决欧几里得群上的 AX = XB 问题》 \cite Park94
- **R. Horaud, F. Dornaika** 《手眼标定》 \cite Horaud95

另一种方法是同时估计旋转和平移（同时解），已实现的方法有：

- **N. Andreff, R. Horaud, B. Espiau** 《在线手眼标定》 \cite Andreff99
- **K. Daniilidis** 《基于双四元数的手眼标定》 \cite Daniilidis98

![image-20250124130938774](.\image\image-20250124130938774.png)

![image-20250124131008104](.\image\image-20250124131008104.png)



所以如果标定的是：

- 眼在手上，输入给`calibrateHandEye` 的参数是
  - $^b T_g$：即从机械臂末端到机械臂基座的变换矩阵（R和t）
  - $^cT_t$：即从标定板到相机的变换矩阵（R和t）
  - 最后得到的是 $^gT_c$：即相机到机械臂末端的变换矩阵
- 眼在手外：输入给 `calibrateHandEye`的参数是
  - $^gT_b$：即从机械臂基座到机械臂末端的变换矩阵（R和t）【也就是根据正向运动学得到的TCP需要求个逆】
  - $^cT_t$：即从标定板到相机的变换矩阵（R和t）
  - 最后得到的是 $^b T _c$：即相机到机械臂基座的变换矩阵
