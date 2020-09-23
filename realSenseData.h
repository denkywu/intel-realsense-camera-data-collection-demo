/**********************************************************************//**
						"class RealSenseData"

	Core of Data Acquisition based on Intel RealSense SDK 2.0 and
		D400 series cameras (e.g. D435i, D435, D415)

@file		realSenseData.h
@author		WD
@date		2020
@brief		Statement for "realSenseData.cpp"
**************************************************************************/
#ifndef _REALSENSE_DATA_H_
#define _REALSENSE_DATA_H_

// Intel RealSense Cross Platform API
#include <librealsense2/rs.hpp>
// OpenCV
#include <opencv2/opencv.hpp>


// 宏: 是否输出显示信息
#define REALSENSEDATA_DEBUG 1


// 自定义类，用来获取 Intel RealSense 相机的 2D/3D 数据
//	(目前已经在 D435i, D435, D415 上经过测试)
//	(理论上只要支持 Intel RealSense SDK 2.0 的设备都可以. 只是要注意某些设备可能有/没有某些接口)
class RealSenseData
{
public:
	// 设备状态
	bool devStart();// 设备启动
	bool devStop();// 设备关闭
	bool isDevConnected() const;// 设备是否连接
	bool isDevReady() const;// 设备是否准备好采集数据
	
	// 数据
	cv::Mat getColorImage_2dCamera();// 2D彩色相机获取的2D彩色图像
	cv::Mat getPointCloud_3dCamera();// 3D相机获取的点云xyz数据
	cv::Mat getTextureImage_3dCamera();// 3D相机获取的点云纹理彩色图像
	cv::Mat getIrLeftImage_3dCamera();// 3D相机获取的 IR left 图像
	cv::Mat getIrRightImage_3dCamera();// 3D相机获取的 IR right 图像
	
	// 相机内参和畸变参数
	// 2D相机
	cv::Matx33f getIntrinsics_2dCamera();// 2D相机-内参
	cv::Matx<float,5,1> getDistortion_2dCamera();// 2D相机-畸变参数
	// 3D相机
	cv::Matx33f getIntrinsics_3dCamera();// 3D相机-内参
	cv::Matx<float,5,1> getDistortion_3dCamera();// 3D相机-畸变参数

	// 外参, get Depth-to-Color Extrinsics
	cv::Matx33f getRotation();
	cv::Matx31f getTranslation();

private:
	int rs2DepthWidthCols_ = 0;// depthWidth
	int rs2DepthHeightRows_ = 0;// depthHeight
	int rs2RgbWidthCols_ = 0;// colorWidth
	int rs2RgbHeightRows_ = 0;// colorHeight
	int rs2InfraredWidthCols_ = 0;// infraredWidth
	int rs2InfraredHeightRows_ = 0;// infraredHeight
	int rs2Fps_ = 0;// fps

	bool isRs2Connected_ = false;
	bool isRs2Ready_ = false;

	rs2::pipeline rs2Pipe_ = {};// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pointcloud rs2PointCloud_ = {};// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pipeline_profile rs2Selection_ = {};

	// 标记
	//		分别放在 getColorImage_2dCamera(), getPointCloud_3dCamera(), getTextureImage_3dCamera(), 
	//			getIrLeftImage_3dCamera(), getIrRightImage_3dCamera() 中，用来判断是否需要重新取数据
	//		每个标记只能对应唯一一个（上面的）成员函数，顺序随意。但一旦确定，不要更改。
	std::vector<bool> isNeedCaptureFlag_ = {true, true, true, true, true};// 注意初值，有几个需要标记的成员函数，就有几个标记
	void changeIsNeedCaptureFlag(size_t idexFlag);// 用来改变标记的状态

	cv::Mat imageColor_ = {};// 2D 彩色图像
	cv::Mat pointCloud_ = {};// 点云 xyz
	cv::Mat imageTexture_ = {};// 点云纹理彩色图像
	cv::Mat imageInfraredLeft_ = {};// 左侧 IR 灰度图
	cv::Mat imageInfraredRight_ = {};// 右侧 IR 灰度图
	// cv::Mat imageColor_ = cv::Mat::zeros(rs2RgbHeightRows_, rs2RgbWidthCols_, CV_8UC3);// 2D 彩色图像
	// cv::Mat pointCloud_ = cv::Mat::zeros(rs2DepthHeightRows_, rs2DepthWidthCols_, CV_32FC3);// 点云 xyz
	// cv::Mat imageTexture_ = cv::Mat::zeros(rs2DepthHeightRows_, rs2DepthWidthCols_, CV_8UC3);// 点云纹理彩色图像
	// cv::Mat imageInfraredLeft_ = cv::Mat::zeros(rs2InfraredHeightRows_, rs2InfraredWidthCols_, CV_8UC1);// 左侧 IR 灰度图
	// cv::Mat imageInfraredRight_ = cv::Mat::zeros(rs2InfraredHeightRows_, rs2InfraredWidthCols_, CV_8UC1);// 右侧 IR 灰度图
	// —— 上面这几个矩阵的大小，得等到 devStart() 中读入了相应图像的 width 和 column 后（相机配置文件）才能知道。所以这里先定义为空。

	// 内参, 畸变系数
	bool isGetColorCameraParams_ = false;
	bool isGetDepthCameraParams_ = false;
	cv::Matx33f intrMat_2dCamera_ = cv::Matx33f::zeros();// 2D相机-内参
	cv::Matx<float,5,1> discoef_2dCamera_ = cv::Matx<float,5,1>::zeros();// 2D相机-畸变参数
	cv::Matx33f intrMat_3dCamera_ = cv::Matx33f::zeros();// 3D相机-内参
	cv::Matx<float,5,1> discoef_3dCamera_ = cv::Matx<float,5,1>::zeros();// 3D相机-畸变参数	

	// 外参
	bool isGetExtrinsics_ = false;
	cv::Matx33f rotationMat_= cv::Matx33f::zeros();
	cv::Matx31f translationMat_ = cv::Matx31f::zeros();

	bool getRealSenseData();// 从设备获取数据
	bool get2dCameraParams();// 获取 2D 的内参和畸变参数
	bool get3dCameraParams();// 获取 3D 的内参和畸变参数
	bool getDepthToColorExtrinsics();// get Depth-to-Color Extrinsics
};
#endif// _REALSENSE_DATA_H_