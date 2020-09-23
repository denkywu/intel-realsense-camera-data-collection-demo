/**********************************************************************//**
		Demo about data acquisition using class RealSenseData

@file		mainDemo.cpp
@author		WD
@date		2020
@brief		Data acquisition using class RealSenseData
**************************************************************************/
// class RealSenseData
#include "realSenseData.h"
// C++
#include <iostream>
#include <string>
// OpenCV
#include <opencv2/opencv.hpp>
// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


// 自定义函数
// convert data of cv::Mat format to PCL pointCloud
bool convertMatToPcl(const cv::Mat& xyzDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
// combine XYZ pointCloud with Texture image, generate xyzrgb cloud
bool combineXyzWithTexture(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud, 
      const cv::Mat& textureImage, 
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud
);


// demo
int main(int argc, char* argv[]) try
{
	// ---------------------------------------------------------------------------------------------------------
	/*
			实例化类，进行数据采集实验
	*/
	RealSenseData cameraDataCapture;

	if (false == cameraDataCapture.devStart())
	{
		std::cerr << "[Error] Failed in starting device." << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "***********************************************" << std::endl;
	std::cout << "Is RS2 device connected or not: " << cameraDataCapture.isDevConnected() << std::endl;
	std::cout << "Is RS2 device ready (can capture Data) or not: " << cameraDataCapture.isDevReady() << std::endl;

	// RealSense SDK中读取 color 即2D相机内参
	auto intrMat_2d = cameraDataCapture.getIntrinsics_2dCamera();// 2D相机-内参
	std::cout << "intrMat_2d: \n" << intrMat_2d << std::endl;
	auto discoef_2d = cameraDataCapture.getDistortion_2dCamera();// 2D相机-畸变参数
	std::cout << "discoef_2d: \n" << discoef_2d << std::endl;
	std::cout << "------" << std::endl;
#if 0
	cv::FileStorage fsColor;
	fsColor.open("colorCameraParams_from_SDK.yaml", cv::FileStorage::WRITE);
	if(!fsColor.isOpened())
	{
		std::cerr << "[Error] failed to open file" << std::endl;
		return EXIT_FAILURE;
	}
	fsColor << "intrMat_2d" << cv::Mat(intrMat_2d, false);
	fsColor << "discoef_2d" << cv::Mat(discoef_2d, false);
	fsColor.release();
	std::cout << "Done: save intrinsics params of COLOR camera from SDK." << std::endl;
#endif

	// RealSense SDK中读取 depth 即3D相机内参
	auto intrMat_3d = cameraDataCapture.getIntrinsics_3dCamera();// 3D相机-内参
	std::cout << "intrMat_3d: \n" << intrMat_3d << std::endl;
	auto discoef_3d  = cameraDataCapture.getDistortion_3dCamera();// 3D相机-畸变参数
	std::cout << "discoef_3d: \n" << discoef_3d << std::endl;
	std::cout << "------" << std::endl;
#if 0
	cv::FileStorage fsDepth;
	fsDepth.open("depthCameraParams_from_SDK.yaml", cv::FileStorage::WRITE);
	if(!fsDepth.isOpened())
	{
		std::cerr << "[Error] failed to open file" << std::endl;
		return EXIT_FAILURE;
	}
	fsDepth << "intrMat_3d" << cv::Mat(intrMat_3d, false);
	fsDepth << "discoef_3d" << cv::Mat(discoef_3d, false);
	fsDepth.release();
	std::cout << "Done: save intrinsics params of DEPTH camera from SDK." << std::endl;
#endif

	// RealSense SDK中读取外参 Depth-to-Color Extrinsics
	auto rotationMat = cameraDataCapture.getRotation();// 外参-旋转矩阵
	std::cout << "rotation: \n" << rotationMat << std::endl;
	auto translationMat  = cameraDataCapture.getTranslation();// 外参-平移向量
	std::cout << "translation: \n" << translationMat << std::endl;
	std::cout << "------" << std::endl;
#if 0
	cv::FileStorage fsExtrinsics;
	fsExtrinsics.open("extrinsics_from_SDK.yaml", cv::FileStorage::WRITE);
	if(!fsExtrinsics.isOpened())
	{
		std::cerr << "[Error] failed to open file" << std::endl;
		return EXIT_FAILURE;
	}
	fsExtrinsics << "rotation" << cv::Mat(rotationMat, false);
	fsExtrinsics << "translation" << cv::Mat(translationMat, false);
	fsExtrinsics.release();
	std::cout << "Done: save extrinsics params from SDK." << std::endl;
#endif


	//
	// 循环获取数据并显示
	//
	int itest = 0;
	while (true) // 一直循环，没有退出条件
	// for ( ; itest != 30; ) // 循环指定次数
	{
		/*
			2D彩色图像
		*/
		cv::Mat colorImage_2dCamera = cameraDataCapture.getColorImage_2dCamera();
		if (0 == itest)
		{
			std::cout << "colorImage_2dCamera: \n"
				<< " - type: " << colorImage_2dCamera.type() << "\n"
				<< " - channel: " << colorImage_2dCamera.channels() << "\n"
				<< " - size: " << colorImage_2dCamera.size
				<< std::endl;
		}
		if (!colorImage_2dCamera.empty())
		{
			cv::imshow("Rgb", colorImage_2dCamera);
		}
		// 保存 2D彩色图像 到本地，见后


		/*
			点云xyz
		*/
		cv::Mat pointCloud_3dCamera = cameraDataCapture.getPointCloud_3dCamera();
		if (0 == itest)
		{
			std::cout << "pointCloud_3dCamera: \n"
				<< " - type: " << pointCloud_3dCamera.type() << "\n"
				<< " - channel: " << pointCloud_3dCamera.channels() << "\n"
				<< " - size: " << pointCloud_3dCamera.size
				<< std::endl;
			// std::cout << pointCloud_3dCamera.elemSize() << std::endl;
		}
		// 点云xyz没有进行可视化
		// 保存 点云xyz 到本地，见后


		// 点云纹理彩色图像
		cv::Mat textureImage_3dCamera = cameraDataCapture.getTextureImage_3dCamera();
		if (0 == itest)
		{
			std::cout << "textureImage_3dCamera: \n"
				<< " - type: " << textureImage_3dCamera.type() << "\n"
				<< " - channel: " << textureImage_3dCamera.channels() << "\n"
				<< " - size: " << textureImage_3dCamera.size
				<< std::endl;
		}
		if (!textureImage_3dCamera.empty())
		{
			cv::imshow("Texture", textureImage_3dCamera);
		}
		// 保存 点云纹理彩色图像 到本地，见后


		// ir 灰度图像，包括左右两幅
		// left
		cv::Mat irLeftImage_3dCamera = cameraDataCapture.getIrLeftImage_3dCamera();
		if (0 == itest)
		{
			std::cout << "irLeftImage_3dCamera: \n"
				<< " - type: " << irLeftImage_3dCamera.type() << "\n"
				<< " - channel: " << irLeftImage_3dCamera.channels() << "\n"
				<< " - size: " << irLeftImage_3dCamera.size
				<< std::endl;
		}
		if (!irLeftImage_3dCamera.empty())
		{
			cv::imshow("Ir_left", irLeftImage_3dCamera);
		}
		// right
		cv::Mat irRightImage_3dCamera = cameraDataCapture.getIrRightImage_3dCamera();
		if (0 == itest)
		{
			std::cout << "irRightImage_3dCamera: \n"
				<< " - type: " << irRightImage_3dCamera.type() << "\n"
				<< " - channel: " << irRightImage_3dCamera.channels() << "\n"
				<< " - size: " << irRightImage_3dCamera.size
				<< std::endl;
		}
		if (!irRightImage_3dCamera.empty())
		{
			cv::imshow("Ir_right", irRightImage_3dCamera);
		}
		// 保存 ir灰度图像 到本地，见后


		cv::waitKey(3);// 配合 cv::imshow 使用

		// std::cout << "Running " << itest << " ..." << std::endl;
		itest++;
	}


	// ---------------------------------------------------------------------------------------------------------
	/*
			再获取最新的一帧数据后，保存文件到本地
	*/
#if 1
	// 设置保存路径
	const std::string pathOutput("Test/");// 注意需要先自行创建文件夹

	// 2D彩色图像
	cv::Mat colorImage_2dCamera = cameraDataCapture.getColorImage_2dCamera();
	if (!colorImage_2dCamera.empty())
	{
		std::string pathColor = pathOutput + "colorImage.png";
		cv::imwrite(pathColor, colorImage_2dCamera);
		std::cout << "Done: save colorImage_2dCamera as png-file." << std::endl;
	}
	else
	{
		std::cout << "[Warning] data of colorImage_2dCamera is NULL." << std::endl;
	}
	
	// 点云
	// 	下面分别保存为 1）xml文件; 2）pcd格式点云文件
	cv::Mat pointCloud_3dCamera = cameraDataCapture.getPointCloud_3dCamera();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (!pointCloud_3dCamera.empty())
	{
		// xml
		std::string pathPointCloud = pathOutput + "pointCloud.xml";
		cv::FileStorage fs_pointCloud(pathPointCloud, cv::FileStorage::WRITE);
		fs_pointCloud << "pointCloud" << pointCloud_3dCamera;// 双引号内绝对不能有空格 !
		fs_pointCloud.release();
		std::cout << "Done: save pointCloud_3dCamera as xml-file." << std::endl;

		// 利用pcl保存为 pcd 文件
		if(false == convertMatToPcl(pointCloud_3dCamera, cloud))
		{
			std::cerr << "[Error] convertMatToPcl()" << std::endl;
			return EXIT_FAILURE;
		}
		std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
		pcl::PCDWriter writer;
		std::string pathPointCloudPcd = pathOutput + "pointCloud.pcd";
		writer.write(pathPointCloudPcd, *cloud, false);
		std::cout << "Done: save pointCloud_3dCamera as pcd-file." << std::endl;
	}
	else
	{
		std::cout << "[Warning] data of pointCloud_3dCamera is NULL." << std::endl;
	}
	
	// 点云纹理彩色图像
	cv::Mat textureImage_3dCamera = cameraDataCapture.getTextureImage_3dCamera();
	if (!textureImage_3dCamera.empty())
	{
		std::string pathTexture = pathOutput + "textureImage.png";
		cv::imwrite(pathTexture, textureImage_3dCamera);
		std::cout << "Done: save textureImage_3dCamera as png-file." << std::endl;
	}
	else
	{
		std::cout << "[Warning] data of textureImage_3dCamera is NULL." << std::endl;
	}

	// 合成点云xyzrgb, 基于点云xyz和点云纹理彩色图像
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzRgbCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	if (!pointCloud_3dCamera.empty() && !textureImage_3dCamera.empty())
	{
		if (false == combineXyzWithTexture(cloud, textureImage_3dCamera, xyzRgbCloud))
		{
			std::cerr << "[Error] combineXyzWithTexture()" << std::endl;
			return EXIT_FAILURE;
		}
		// 利用pcl保存为 ply 文件
		std::string pathXyzRgbCloud = pathOutput + "pointCloudWithRgb.ply";
		pcl::io::savePLYFile<pcl::PointXYZRGB>(pathXyzRgbCloud, *xyzRgbCloud);
		std::cout << "Done: save xyzRgbCloud as ply-file." << std::endl;
	}
	else
	{
		std::cout << "[Waring] cannot generate xyzRgbCloud" << std::endl;
	}


	// ir灰度图像
	// left
	cv::Mat irLeftImage_3dCamera = cameraDataCapture.getIrLeftImage_3dCamera();
	if (!irLeftImage_3dCamera.empty())
	{
		std::string pathIrLeft = pathOutput + "irLeftImage.png";
		cv::imwrite(pathIrLeft, irLeftImage_3dCamera);
		std::cout << "Done: save irLeftImage_3dCamera as png-file." << std::endl;
	}
	else
	{
		std::cout << "[Warning] data of irLeftImage_3dCamera is NULL." << std::endl;
	}
	// right
	cv::Mat irRightImage_3dCamera = cameraDataCapture.getIrRightImage_3dCamera();
	if (!irRightImage_3dCamera.empty())
	{
		std::string pathIrRight = pathOutput + "irRightImage.png";
		cv::imwrite(pathIrRight, irRightImage_3dCamera);
		std::cout << "Done: save irRightImage_3dCamera as png-file." << std::endl;
	}
	else
	{
		std::cout << "[Warning] data of irRightImage_3dCamera is NULL." << std::endl;
	}
#endif// 保存文件


	// ---------------------------------------------------------------------------------------------------------
	/*
	 		关闭设备
	*/
	if (false == cameraDataCapture.devStop()) // TODO. 目前返回值一定是true，内部未实现
	{
		std::cerr << "[Error] Stop device failed." << std::endl;
		return EXIT_FAILURE;
	}
	else
	{
		std::cout << "Device stopped. " << std::endl;
	}


	std::cout << "----------------------------------" << std::endl;
	std::cout << "------------- closed -------------" << std::endl;
	std::cout << "----------------------------------" << std::endl;
	
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}


// convert cv::Mat format to pcl::PointCloud<pcl::PointXYZ>::Ptr
bool convertMatToPcl(const cv::Mat& xyzDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // xyzDepth 的数据类型必须是 CV_32FC3
    if(21 != xyzDepth.type())
    {
        std::cerr << "Error in convertMatToPcl()." << std::endl;
		std::cerr << "The type of input data(cv::Mat) must be CV_32FC3." << std::endl;
        return false;
    }
	// cv::Mat数据大小
	auto depthWidth = xyzDepth.cols;
	auto depthHeight = xyzDepth.rows;

	cloud->width = static_cast<uint32_t>(depthWidth);
	cloud->height = static_cast<uint32_t>(depthHeight);
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);

	pcl::PointXYZ* pt = &cloud->points[0];
	for (int iRows = 0; iRows < depthHeight; iRows++)
	{
		for (int iCols = 0; iCols < depthWidth; iCols++, pt++)
		{
			pt->x = xyzDepth.at<cv::Vec3f>(iRows, iCols)[0];
			pt->y = xyzDepth.at<cv::Vec3f>(iRows, iCols)[1];
			pt->z = xyzDepth.at<cv::Vec3f>(iRows, iCols)[2];
		}
	}
	return true;
}


// generate XYZRGB pointCloud by combining XYZ pointCloud with Texture image
//	[IN] pointCloud xyz information in PCL format
//	[IN] texture image (CV_8UC3)
//	[OUT] pointCloud xyzRgb information in PCL format
bool combineXyzWithTexture(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud, 
    const cv::Mat& textureImage, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud
)
{
    // textureImage 的数据类型必须是 CV_8UC3
    if(16 != textureImage.type())
    {
        std::cerr << "[Error] The type of textureImage(cv::Mat) must be CV_8UC3." << std::endl;
        return false;
    }

    xyzRgbCloud->width = xyzCloud->width;
    xyzRgbCloud->height = xyzCloud->height;
    xyzRgbCloud->is_dense = false;
    xyzRgbCloud->points.resize(xyzRgbCloud->height * xyzRgbCloud->width);

    pcl::PointXYZRGB* pt = &xyzRgbCloud->points[0];
    for (int i = 0; i < xyzCloud->width * xyzCloud->height; i++)
    {
        if (xyzCloud->points[i].z < 1e-5)// 无效点云用 0 (z=0) 表示的，新点云中改为 z=NaN 表示
        {
            pt->x = std::numeric_limits<float>::quiet_NaN();
            pt->y = std::numeric_limits<float>::quiet_NaN();
            pt->z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            pt->x = xyzCloud->points[i].x;
            pt->y = xyzCloud->points[i].y;
            pt->z = xyzCloud->points[i].z;
        }

        pt->b = textureImage.at<cv::Vec3b>(i)[0];// CV_8UC3, 三个通道依次表示 bgr
        pt->g = textureImage.at<cv::Vec3b>(i)[1];
        pt->r = textureImage.at<cv::Vec3b>(i)[2];

        pt++;
    }
    return true;
}