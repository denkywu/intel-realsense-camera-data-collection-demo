/**********************************************************************//**
						"class RealSenseData"

	Core of Data Acquisition based on Intel RealSense SDK 2.0 and
		D400 series cameras (e.g. D435i, D435, D415)

@file		realSenseData.cpp
@author		WD
@date		2020
@brief		Data Acquisition based on Intel RealSense device
**************************************************************************/
#include "realSenseData.h"
// Intel RealSense Cross Platform API
#include <librealsense2/rs.hpp>
// C++
#include <iostream>
#include <vector>
#include <math.h>
// opencv
#include <opencv2/opencv.hpp>


// 设备启动: 设置相机参数，打开默认设备，运行相机
bool RealSenseData::devStart()
{
	// ---------------------------------------------------------------------------------------------------------
	/*
				0 - 检查是否连接设备
	*/
	// The context represents the current platform with respect to connected devices
	rs2::context ctx;
	// Using the context we can get all connected devices in a device list
	rs2::device_list devices = ctx.query_devices();
	if (0 == devices.size())
	{
		std::cerr << "[Error] No device connected, please connect a RealSense device." << std::endl;
		this->isRs2Connected_ = false;
        this->isRs2Ready_ = false;
		return false;
	}
	else
	{
		for (auto&& dev : devices)// 打印所有已连接的设备序列号
		{
			std::string devSerial;
			devSerial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);// Device serial number
			std::cout << "RealSense device serial number: " << devSerial << std::endl;
			
			std::cout << "Primary firmware version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
			std::cout << "Recommended firmware version: " << dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION) << std::endl;
		}
	}
	this->isRs2Connected_ = true;// 设备已连接


	// ---------------------------------------------------------------------------------------------------------
	/*
				1 - 读配置文件 "ConfigCameraRealSense.yaml"，完成相机参数等配置
	*/
	cv::FileStorage fsReadConfig("ConfigCameraRealSense.yaml", cv::FileStorage::READ);
	if (!fsReadConfig.isOpened())
	{
		std::cerr << "[Error] Failed: read ConfigCameraRealSense.yaml." << std::endl;
        this->isRs2Ready_ = false;
		return false;
	}
	else
	{
		// depth
		fsReadConfig["depth_width"] >> this->rs2DepthWidthCols_;
		fsReadConfig["depth_height"] >> this->rs2DepthHeightRows_;
		// color
		fsReadConfig["color_width"] >> this->rs2RgbWidthCols_;
		fsReadConfig["color_height"] >> this->rs2RgbHeightRows_;
		// infrared
		fsReadConfig["infrared_width"] >> this->rs2InfraredWidthCols_;
		fsReadConfig["infrared_height"] >> this->rs2InfraredHeightRows_;
		// fps
		fsReadConfig["camera_fps"] >> this->rs2Fps_;

#if REALSENSEDATA_DEBUG == 1
		std::cout << "depthWidth: " << this->rs2DepthWidthCols_ << "\n"
			<< "depthHeight: " << this->rs2DepthHeightRows_ << "\n"
			<< "infraredWidth: " << this->rs2InfraredWidthCols_ << "\n"
			<< "infraredHeight: " << this->rs2InfraredHeightRows_ << "\n"
			<< "colorWidth: " << this->rs2RgbWidthCols_ << "\n"
			<< "colorHeight: " << this->rs2RgbHeightRows_ << "\n"
			<< "fps: " << this->rs2Fps_ << std::endl;
		std::cout << "Succeed: read ConfigCameraRealSense.yaml." << std::endl;
#endif
	}
	fsReadConfig.release();


	// ---------------------------------------------------------------------------------------------------------
	/*
				2 - Stream configuration
	*/
	// 以下 2选1
	//=============================================
	// Choose 1: Default recommended configuration
	//=============================================
	// The default video configuration contains Depth and Color streams（WD: 实验也表明不包括 infrared streams）
	// If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
	// this->rs2Pipe_.start();

	//===============================
	// Choose 2: Non default profile
	//===============================
	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;
	// depth
	cfg.enable_stream(RS2_STREAM_DEPTH, this->rs2DepthWidthCols_, this->rs2DepthHeightRows_, RS2_FORMAT_Z16, this->rs2Fps_);
	// color
	cfg.enable_stream(RS2_STREAM_COLOR, this->rs2RgbWidthCols_, this->rs2RgbHeightRows_, RS2_FORMAT_RGB8, this->rs2Fps_);
	// infrared
	cfg.enable_stream(RS2_STREAM_INFRARED, 1, this->rs2InfraredWidthCols_, this->rs2InfraredHeightRows_, RS2_FORMAT_Y8, this->rs2Fps_);// left IR
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, this->rs2InfraredWidthCols_, this->rs2InfraredHeightRows_, RS2_FORMAT_Y8, this->rs2Fps_);// right IR
	
	this->rs2Selection_ = this->rs2Pipe_.start(cfg);
    // 用来设置 laser power
#if 0
	rs2::device selected_device = this->rs2Selection_.get_device();
	auto depth_sensor = selected_device.first<rs2::depth_sensor>();
	if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))// 设置 EMITTER
	{
		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
		// depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
	}
	if (depth_sensor.supports(RS2_OPTION_LASER_POWER))// 设置 LASER POWER
	{
		auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
		// std::cout << range.max << std::endl;
		// std::cout << range.min << std::endl;
		// std::cout << range.def << std::endl;
		// std::cout << range.step << std::endl;

		// depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
		depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.def); // default value
		// depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
	}
#endif


	// ---------------------------------------------------------------------------------------------------------
	/*
				3 - Capture several frames to give auto-exposure	
	*/
	for (int iframes = 0; iframes < 30; iframes++)
	{
		this->rs2Pipe_.wait_for_frames();
	}


	// ---------------------------------------------------------------------------------------------------------
	/*
				4 - 给 数据矩阵 设置相应的大小
	*/
    this->imageColor_ = cv::Mat::zeros(this->rs2RgbHeightRows_, this->rs2RgbWidthCols_, CV_8UC3);// 2D 彩色图像
	this->pointCloud_ = cv::Mat::zeros(this->rs2DepthHeightRows_, this->rs2DepthWidthCols_, CV_32FC3);// 点云 xyz
	this->imageTexture_ = cv::Mat::zeros(this->rs2DepthHeightRows_, this->rs2DepthWidthCols_, CV_8UC3);// 点云纹理彩色图像
	this->imageInfraredLeft_ = cv::Mat::zeros(this->rs2InfraredHeightRows_, this->rs2InfraredWidthCols_, CV_8UC1);// 左侧 IR 灰度图
	this->imageInfraredRight_ = cv::Mat::zeros(this->rs2InfraredHeightRows_, this->rs2InfraredWidthCols_, CV_8UC1);// 右侧 IR 灰度图


#if REALSENSEDATA_DEBUG == 1
	std::cout << "----------------------------------" << std::endl;
	std::cout << "----- Have Started RealSense. -----" << std::endl;
	std::cout << "----------------------------------" << std::endl;
#endif// REALSENSEDATA_DEBUG

    this->isRs2Ready_ = true;
	return true;// 设备已启动
}


// 设备关闭: 释放资源，关闭设备
bool RealSenseData::devStop()
{
    // TODO

	// ---------------------------------------------------------------------------------------------------------
	/*
				1 - 释放资源
	*/	

	// ---------------------------------------------------------------------------------------------------------
	/*
				2 - 关闭设备
	*/

    this->isRs2Ready_ = false;// 既然成功停止设备，那“是否准备好采集数据”就应该置为false了。
	return true;
}


// change "isNeedCaptureFlag_"
//  [IN] index of Flag which need NOT to be changed (不需要改变的标记的索引)
void RealSenseData::changeIsNeedCaptureFlag(size_t idexFlag)
{
	for (size_t iChange = 0; iChange != this->isNeedCaptureFlag_.size(); iChange++)
	{
		if (iChange != idexFlag)
		{
			this->isNeedCaptureFlag_[iChange] = false;// 将除了索引 idexFlag 外的所有标记全部置为 false
		}
	}
}


// 设备是否连接
bool RealSenseData::isDevConnected() const
{
    return this->isRs2Connected_;
}


// 设备是否准备好采集数据
bool RealSenseData::isDevReady() const
{
    return this->isRs2Ready_;
}


// 2D彩色相机获取的2D彩色图像
//  [OUT] imageColor_ in cv::Mat (数据类型 CV_8UC3)
cv::Mat RealSenseData::getColorImage_2dCamera() // isNeedCaptureFlag_[0]
{
	// 逻辑判断
	if (true == this->isNeedCaptureFlag_[0])
	{
		// 获取数据（更新数据）
		bool isGetRealSenseData = getRealSenseData();
		if (false == isGetRealSenseData)// 如果获取失败
		{
			std::cerr << "[Error] getRealSenseData() Failed!" << std::endl;
			return cv::Mat{};
		}

		// 本标记保持不变，其余必须修改成 false
		changeIsNeedCaptureFlag(0);
	}
	else
	{
		// 不需要获取数据（更新数据）：将本标记修改成 true，其余不用管
		this->isNeedCaptureFlag_[0] = true;
	}

	// 返回值
	return this->imageColor_;
}


// 3D相机获取的点云xyz数据
//  [OUT] pointCloud in cv::Mat (数据类型 CV_32FC3)
cv::Mat RealSenseData::getPointCloud_3dCamera() // isNeedCaptureFlag_[1]
{
	// 逻辑判断
	if (true == this->isNeedCaptureFlag_[1])
	{
		// 获取数据（更新数据）
		bool isGetRealSenseData = getRealSenseData();
		if (false == isGetRealSenseData)// 如果获取失败
		{
			std::cerr << "[Error] getRealSenseData() Failed!" << std::endl;
			return cv::Mat{};
		}

		// 本标记保持不变，其余必须修改成 false
		changeIsNeedCaptureFlag(1);
	}
	else
	{
		// 不需要获取数据（更新数据）：将本标记修改成 true，其余不用管
		this->isNeedCaptureFlag_[1] = true;
	}

	// 返回值
	return this->pointCloud_;
}


// 3D相机获取的点云纹理彩色图像
//  [OUT] imageTexture_ in cv::Mat (数据类型 CV_8UC3)
cv::Mat RealSenseData::getTextureImage_3dCamera() // isNeedCaptureFlag_[2]
{
	// 逻辑判断
	if (true == this->isNeedCaptureFlag_[2])
	{
		// 获取数据（更新数据）
		bool isGetRealSenseData = getRealSenseData();
		if (false == isGetRealSenseData)// 如果获取失败
		{
			std::cerr << "[Error] getRealSenseData() Failed!" << std::endl;
			return cv::Mat{};
		}

		// 本标记保持不变，其余必须修改成 false
		changeIsNeedCaptureFlag(2);
	}
	else
	{
		// 不需要获取数据（更新数据）：将本标记修改成 true，其余不用管
		this->isNeedCaptureFlag_[2] = true;
	}

	// 返回值
	return this->imageTexture_;
}


// 3D相机获取的 IR left 图像
//  [OUT] imageInfraredLeft_ in cv::Mat (数据类型 CV_8UC1)
cv::Mat RealSenseData::getIrLeftImage_3dCamera() // isNeedCaptureFlag_[3]
{
	// 逻辑判断
	if (true == this->isNeedCaptureFlag_[3])
	{
		// 获取数据（更新数据）
		bool isGetRealSenseData = getRealSenseData();
		if (false == isGetRealSenseData)// 如果获取失败
		{
			std::cerr << "[Error] getRealSenseData() Failed!" << std::endl;
			return cv::Mat{};
		}

		// 本标记保持不变，其余必须修改成 false
		changeIsNeedCaptureFlag(3);
	}
	else
	{
		// 不需要获取数据（更新数据）：将本标记修改成 true，其余不用管
		this->isNeedCaptureFlag_[3] = true;
	}

	// 返回值
	return this->imageInfraredLeft_;
}


// 3D相机获取的 IR right 图像
//  [OUT] imageInfraredRight_ in cv::Mat (数据类型 CV_8UC1)
cv::Mat RealSenseData::getIrRightImage_3dCamera() // isNeedCaptureFlag_[4]
{
	// 逻辑判断
	if (true == this->isNeedCaptureFlag_[4])
	{
		// 获取数据（更新数据）
		bool isGetRealSenseData = getRealSenseData();
		if (false == isGetRealSenseData)// 如果获取失败
		{
			std::cerr << "[Error] getRealSenseData() Failed!" << std::endl;
			return cv::Mat{};
		}

		// 本标记保持不变，其余必须修改成 false
		changeIsNeedCaptureFlag(4);
	}
	else
	{
		// 不需要获取数据（更新数据）：将本标记修改成 true，其余不用管
		this->isNeedCaptureFlag_[4] = true;
	}

	// 返回值
	return this->imageInfraredRight_;
}


// 2D相机-内参
//  [OUT] intrMat_2dCamera_
cv::Matx33f RealSenseData::getIntrinsics_2dCamera()
{
	if (true == get2dCameraParams())
	{
		return this->intrMat_2dCamera_;
	}
	else
	{
		return cv::Matx33f::zeros();
	}
}


// 2D相机-畸变参数
//  [OUT] discoef_2dCamera_
cv::Matx<float,5,1> RealSenseData::getDistortion_2dCamera()
{
	if (true == get2dCameraParams())
	{
		return this->discoef_2dCamera_;
	}
	else
	{
		return cv::Matx<float,5,1>::zeros();
	}
}


// 3D相机-内参
//  [OUT] intrMat_3dCamera_
cv::Matx33f RealSenseData::getIntrinsics_3dCamera()
{
	if (true == get3dCameraParams())
	{
		return this->intrMat_3dCamera_;
	}
	else
	{
		return cv::Matx33f::zeros();
	}
}


// 3D相机-畸变参数
//  [OUT] discoef_3dCamera_
cv::Matx<float,5,1> RealSenseData::getDistortion_3dCamera()
{
	if (true == get3dCameraParams())
	{
		return this->discoef_3dCamera_;
	}
	else
	{
		return cv::Matx<float,5,1>::zeros();
	}
}


// Extrinsics, rotation matrix
//  [OUT]  rotationMat_
cv::Matx33f RealSenseData::getRotation()
{
	if (true == getDepthToColorExtrinsics())
	{
		return this->rotationMat_;
	}
	else
	{
		return cv::Matx33f::zeros();
	}
}


// Extrinsics, translation vector, in meters
//  [OUT]  translationMat_
cv::Matx31f RealSenseData::getTranslation()
{
	if (true == getDepthToColorExtrinsics())
	{
		return this->translationMat_;
	}
	else
	{
		return cv::Matx31f::zeros();
	}
}


// 从设备获取数据
bool RealSenseData::getRealSenseData()
{
	if(false == this->isRs2Ready_)
	{
		std::cerr << "[Error] RealSense device is not ready. Please confirm!" << std::endl;
		return false;
	}


	// ---------------------------------------------------------------------------------------------------------
	/*
				0 - 获取新的多源数据帧
	*/
	// 获取数据帧: Wait for the next set of frames from the camera
	auto rs2_Frameset = this->rs2Pipe_.wait_for_frames();// Set of time synchronized frames, one from each active stream


	// ---------------------------------------------------------------------------------------------------------
	/*
				(可选) post-processing

		参考: 
			example - measure - "rs-measure.cpp", 
			example - post-processing - "rs-post-processing.cpp"
			"https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md"
	*/
#if 0
	//
	// Declare filters
	//
    rs2::decimation_filter dec_filter;	// Decimation - reduces depth frame density
	rs2::threshold_filter thr_filter;  	// Threshold  - removes values outside recommended range
	rs2::spatial_filter spat_filter;	// Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;	// Temporal   - reduces temporal noise
    rs2::disparity_transform depth2disparity(true);// Declare disparity transform from depth to disparity and vice versa
    rs2::disparity_transform disparity2depth(false);

	//
	// Configure filter parameters
	//
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	// auto range = dec_filter.get_option_range(RS2_OPTION_FILTER_MAGNITUDE);
	// std::cout << "range.max: " << range.max << "\n" 
	// 	 << "range.min: " << range.min << "\n" 
	// 	 << "range.def: " << range.def << "\n" 
	// 	 << "range.step: " << range.step << std::endl;
	thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.15f);
	thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 4.f);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
	temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);//* \param[in] persistence_control - A set of predefined rules (masks) that govern when missing pixels will be replaced with the last valid value so that the data will remain persistent over time
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);

    // // Spatially align all streams to depth viewport
    // // We do this because:
    // //   a. Usually depth has wider FOV, and we only really need depth for this demo
    // //   b. We don't want to introduce new holes
    // rs2::align align_to(RS2_STREAM_DEPTH);

	//
	// Apply filters
	//
	// // First make the frames spatially aligned
	// rs2_Frameset = rs2_Frameset.apply_filter(align_to);

	// // Decimation will reduce the resultion of the depth image, closing small holes and speeding-up the algorithm
	// rs2_Frameset = rs2_Frameset.apply_filter(dec_filter);

	// To make sure far-away objects are filtered proportionally， we try to switch to disparity domain
	rs2_Frameset = rs2_Frameset.apply_filter(depth2disparity);

	// Apply spatial filtering
	rs2_Frameset = rs2_Frameset.apply_filter(spat_filter);

	// Apply temporal filtering
	rs2_Frameset = rs2_Frameset.apply_filter(temp_filter);

	// If we are in disparity domain, switch back to depth
	rs2_Frameset = rs2_Frameset.apply_filter(disparity2depth);
#endif// post-processing


	// ---------------------------------------------------------------------------------------------------------
	/*
				1 - 分离各个数据帧
	*/
	// depth
	auto rs2_DepthFrame = rs2_Frameset.get_depth_frame();
	// color
	auto rs2_ColorFrame = rs2_Frameset.get_color_frame();
	// infrared
	rs2::video_frame rs2_IrFrameLeft = rs2_Frameset.get_infrared_frame(1);
	rs2::video_frame rs2_IrFrameRight = rs2_Frameset.get_infrared_frame(2);


	// ---------------------------------------------------------------------------------------------------------
	/*
				2 - 点云 xyz, pointCloud
				3 - 点云纹理彩色图像, texture image, bgr 格式

		注: 参考"rs_export.hpp"文件中 void export_to_ply(points p, video_frame color){} 进行修改
	*/
	this->rs2PointCloud_.map_to(rs2_ColorFrame);// Tell pointcloud object to map to this frame - as texture.

	// Generate the pointcloud and texture mappings
	rs2::points rs2_Points;
    rs2_Points = this->rs2PointCloud_.calculate(rs2_DepthFrame);

	// 检查点云数据分辨率: 如果数据的尺寸和预设的像素不相等，报错
	// std::cout << "pointCloud size: " << rs2_Points.size() << std::endl;/// 输出点云的size，必须等于配置文件中的设定值
	if (this->rs2DepthWidthCols_ * this->rs2DepthHeightRows_ != rs2_Points.size())
	{
		std::cerr << "[Error] size of pointCloud is NOT equal to the configuration" << std::endl;
		return false;
	}

	const auto pVertex = rs2_Points.get_vertices();
	const auto ColorFrame_data = reinterpret_cast<const uint8_t*>(rs2_ColorFrame.get_data());
	const auto ColorFrame_bytes_per_pixel = rs2_ColorFrame.get_bytes_per_pixel();
	const auto ColorFrame_stride_in_bytes = rs2_ColorFrame.get_stride_in_bytes();

    // 获取点云xyz和点云纹理彩色图像
	const auto pTexture = rs2_Points.get_texture_coordinates();
	for (int iRows = 0; iRows != this->rs2DepthHeightRows_; iRows++)
	{
		for (int iCols = 0; iCols != this->rs2DepthWidthCols_; iCols++)
		{
			int iNum = iRows * this->rs2DepthWidthCols_ + iCols;// 最内层循环，目前进行到的总点数
			const static int w_ColorFrame = rs2_ColorFrame.get_width();
			const static int h_ColorFrame = rs2_ColorFrame.get_height();
			
			if (pVertex[iNum].z < 1e-5)// 无效点云用 0 (z=0) 表示的, 改为用 z=NaN 表示
			{
				// 点云
				this->pointCloud_.at<cv::Vec3f>(iRows, iCols)[0] = std::numeric_limits<float>::quiet_NaN();
				this->pointCloud_.at<cv::Vec3f>(iRows, iCols)[1] = std::numeric_limits<float>::quiet_NaN();
				this->pointCloud_.at<cv::Vec3f>(iRows, iCols)[2] = std::numeric_limits<float>::quiet_NaN();
				// 纹理
				this->imageTexture_.at<cv::Vec3b>(iRows, iCols)[0] = 0;
				this->imageTexture_.at<cv::Vec3b>(iRows, iCols)[1] = 0;
				this->imageTexture_.at<cv::Vec3b>(iRows, iCols)[2] = 0;
			}
			else
			{
				// 点云
				this->pointCloud_.at<cv::Vec3f>(iRows, iCols)[0] = pVertex[iNum].x;
				this->pointCloud_.at<cv::Vec3f>(iRows, iCols)[1] = pVertex[iNum].y;
				this->pointCloud_.at<cv::Vec3f>(iRows, iCols)[2] = pVertex[iNum].z;
				// 纹理
				int x = std::min(
					std::max(int(pTexture[iNum].u * w_ColorFrame + .5f), 0), 
					w_ColorFrame - 1
				);
				int y = std::min(
					std::max(int(pTexture[iNum].v * h_ColorFrame + .5f), 0), 
					h_ColorFrame - 1
				);
				int idx_texture = x * ColorFrame_bytes_per_pixel + y * ColorFrame_stride_in_bytes;
				this->imageTexture_.at<cv::Vec3b>(iRows, iCols)[2] = ColorFrame_data[idx_texture];// r 通道
				this->imageTexture_.at<cv::Vec3b>(iRows, iCols)[1] = ColorFrame_data[idx_texture + 1];// g 通道
				this->imageTexture_.at<cv::Vec3b>(iRows, iCols)[0] = ColorFrame_data[idx_texture + 2];// b 通道
			}
		}
	}


	// ---------------------------------------------------------------------------------------------------------
	/*
				4 - color, 2D 彩色图像, bgr 格式

		注: 参考"rs_export.hpp"文件中 void export_to_ply(points p, video_frame color){} 进行修改
	*/
	for (int iRows = 0; iRows != this->rs2RgbHeightRows_; iRows++)
	{
		for (int iCols = 0; iCols != this->rs2RgbWidthCols_; iCols++)
		{
			int idx_ColorFrame = iCols * ColorFrame_bytes_per_pixel + iRows * ColorFrame_stride_in_bytes;
			this->imageColor_.at<cv::Vec3b>(iRows, iCols)[2] = ColorFrame_data[idx_ColorFrame];// r 通道
			this->imageColor_.at<cv::Vec3b>(iRows, iCols)[1] = ColorFrame_data[idx_ColorFrame + 1];// g 通道
			this->imageColor_.at<cv::Vec3b>(iRows, iCols)[0] = ColorFrame_data[idx_ColorFrame + 2];// b 通道
		}
	}


	// ---------------------------------------------------------------------------------------------------------
	/*
				5 - IR 灰度图，和深度图是对齐的
	*/
	this->imageInfraredLeft_ = cv::Mat(cv::Size(this->rs2InfraredWidthCols_, this->rs2InfraredHeightRows_), 
        CV_8UC1, (void*)rs2_IrFrameLeft.get_data());
	this->imageInfraredRight_ = cv::Mat(cv::Size(this->rs2InfraredWidthCols_, this->rs2InfraredHeightRows_), 
        CV_8UC1, (void*)rs2_IrFrameRight.get_data());


	// ---------------------------------------------------------------------------------------------------------
	/*
			释放资源，准备进行下一次数据采集
	*/
	// TODO
	// 目前的测试来看，长时间运行并没有出现内存泄漏问题.

	
	return true;// 数据获取成功
}


// 获取 2D 的内参和畸变参数
//  Ref: https://github.com/IntelRealSense/librealsense/wiki/API-How-To#get-video-stream-intrinsics
bool RealSenseData::get2dCameraParams()
{
	if (false == this->isGetColorCameraParams_)// 首次则需先获取
	{
		if(false == this->isRs2Ready_)
		{
			std::cerr << "[Error] RealSense device is not ready. Please confirm!" << std::endl;
			return false;
		}

		auto color_stream = this->rs2Selection_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
		auto colorCameraParams = color_stream.get_intrinsics();
#if 0
		auto resolution = std::make_pair(color_stream.width(), color_stream.height());
		auto principal_point = std::make_pair(colorCameraParams.ppx, colorCameraParams.ppy);
		auto focal_length = std::make_pair(colorCameraParams.fx, colorCameraParams.fy);
		rs2_distortion model = colorCameraParams.model;
#endif

		// 内参矩阵
		this->intrMat_2dCamera_ = cv::Matx33f(
			colorCameraParams.fx, 		0.0, 						colorCameraParams.ppx, 
			0.0, 						colorCameraParams.fy, 		colorCameraParams.ppy, 
			0.0, 						0.0, 						1.0 
        );

		// 畸变矩阵
		this->discoef_2dCamera_ = cv::Matx<float,5,1>(
			colorCameraParams.coeffs[0], //k1,
			colorCameraParams.coeffs[1], //k2,
			colorCameraParams.coeffs[2], //p1,
			colorCameraParams.coeffs[3], //p2,
			colorCameraParams.coeffs[4]  //k3
		);

		this->isGetColorCameraParams_ = true;
		return true;
	}
	else
	{
		return true;
	}
}


// 获取 3D 的内参和畸变参数
//  Ref: https://github.com/IntelRealSense/librealsense/wiki/API-How-To#get-video-stream-intrinsics
bool RealSenseData::get3dCameraParams()
{
	if (false == this->isGetDepthCameraParams_)// 首次则需先获取
	{
		if(false == this->isRs2Ready_)
		{
			std::cerr << "[Error] RealSense device is not ready. Please confirm!" << std::endl;
			return false;
		}

		auto depth_stream = this->rs2Selection_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
		auto depthCameraParams = depth_stream.get_intrinsics();
#if 0
		auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
		auto principal_point = std::make_pair(depthCameraParams.ppx, depthCameraParams.ppy);
		auto focal_length = std::make_pair(depthCameraParams.fx, depthCameraParams.fy);
		rs2_distortion model = depthCameraParams.model;
#endif

		// 内参矩阵
		this->intrMat_3dCamera_ = cv::Matx33f(
			depthCameraParams.fx, 		0.0, 						depthCameraParams.ppx, 
			0.0, 						depthCameraParams.fy, 		depthCameraParams.ppy, 
			0.0, 						0.0, 						1.0 
		);

		// 畸变矩阵
		this->discoef_3dCamera_ = cv::Matx<float,5,1>(
			depthCameraParams.coeffs[0], //k1,
			depthCameraParams.coeffs[1], //k2,
			depthCameraParams.coeffs[2], //p1,
			depthCameraParams.coeffs[3], //p2,
			depthCameraParams.coeffs[4]  //k3
		);
		
		this->isGetDepthCameraParams_ = true;
		return true;
	}
	else
	{
		return true;
	}
}


// get Depth-to-Color Extrinsics
//  Ref: https://github.com/IntelRealSense/librealsense/wiki/API-How-To
bool RealSenseData::getDepthToColorExtrinsics()
{
	if (false == this->isGetExtrinsics_)// 首次则需先获取
	{
		if(false == this->isRs2Ready_)
		{
			std::cerr << "[Error] RealSense device is not ready. Please confirm!" << std::endl;
			return false;
		}

		auto depth_stream = this->rs2Selection_.get_stream(RS2_STREAM_DEPTH);
		auto color_stream = this->rs2Selection_.get_stream(RS2_STREAM_COLOR);
		rs2_extrinsics rs2Extrinsics = depth_stream.get_extrinsics_to(color_stream);

		// // Apply extrinsics to the origin
		// float origin[3] { 0.f, 0.f, 0.f };
		// float target[3];
		// rs2_transform_point_to_point(target, &e, origin);

		this->rotationMat_ = cv::Matx33f(
			rs2Extrinsics.rotation[0],	rs2Extrinsics.rotation[3],	rs2Extrinsics.rotation[6], 
			rs2Extrinsics.rotation[1], 	rs2Extrinsics.rotation[4], 	rs2Extrinsics.rotation[7], 
			rs2Extrinsics.rotation[2],	rs2Extrinsics.rotation[5],	rs2Extrinsics.rotation[8]
		);
		this->translationMat_ = cv::Matx31f(
			rs2Extrinsics.translation[0], 
			rs2Extrinsics.translation[1], 
			rs2Extrinsics.translation[2]
		);

		this->isGetExtrinsics_ = true;
		return true;
	}
	else
	{
		return true;
	}
}