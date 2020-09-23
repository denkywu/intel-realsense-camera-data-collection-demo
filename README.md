
# Example
# 通过 Intel RealSense 设备获取2D图像、3D点云等原始数据

Based on Intel RealSense SDK 2.0.  
Test on Intel RealSense D400 series devices (depth cameras), including Intel RealSense depth camera D435i, D435, D415.  
官网: https://www.intelrealsense.com/zh-hans/  
github: https://github.com/IntelRealSense/librealsense  

可获取以下数据:  
1. color image，2D彩色图像
2. 点云 xyz
3. texture image，点云纹理彩色图像，与点云xyz（或者说深度图像）是对齐的
4. 点云 xyzrgb，将2和3组合得到
5. ir left image，左侧ir灰度图像
6. ir right image，右侧ir灰度图像

备注：运行demo时需要把文件 "ConfigCameraRealSense.yaml" 拷贝到可执行文件所在目录。  

