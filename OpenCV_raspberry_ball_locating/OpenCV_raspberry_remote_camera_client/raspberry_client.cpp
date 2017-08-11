#include <ctime>
#include <iostream>
#include <stdio.h>
#include <raspicam_cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

#include "SocketMatTransmissionClient.h"  

#include "my_opencv.h"
#include "my_math.h"
#include <math.h>
#include <algorithm>
#include "uart_num.h"

#include "wiringPi.h"
#include "wiringSerial.h"


using namespace cv;
using namespace std;

//#define STDIO_DEBUG
//#define SOCKET_SEND_IMAGE
//#define FINAL_RELEASE


int main(int argc, char **argv)
{
#ifndef FINAL_RELEASE
	system("sudo killall ball");	  
#endif // !FINAL_RELEASE

	piHiPri(99);//最高优先级
	///声明变量
#ifdef SOCKET_SEND_IMAGE
	cout << "socket connecting..." << endl;
	SocketMatTransmissionClient socketMat;
	socketMat.begin("192.168.2.100");
#endif // SOCKET_SEND_IMAGE
	//摄像头
	raspicam::RaspiCam_Cv cam;
	//参数设置
	float imRawFactor = 0.2;
	cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, cam.get(CV_CAP_PROP_FRAME_WIDTH) * imRawFactor);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, cam.get(CV_CAP_PROP_FRAME_HEIGHT) * imRawFactor);
	int imRawH = cam.get(CV_CAP_PROP_FRAME_HEIGHT),
		imRawW = cam.get(CV_CAP_PROP_FRAME_WIDTH);
	Mat imRaw;
	
	//畸变矫正
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = 7.428373545192941e02*imRawFactor;
	cameraMatrix.at<double>(0, 1) = 0.950790070584049*imRawFactor;
	cameraMatrix.at<double>(0, 2) = 6.631483964605850e02*imRawFactor;
	cameraMatrix.at<double>(1, 1) = 7.427433947316842e02*imRawFactor;
	cameraMatrix.at<double>(1, 2) = 4.342201658042163e02*imRawFactor;
	
	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
	distCoeffs.at<double>(0, 0) = -0.290711633529124;
	distCoeffs.at<double>(1, 0) = 0.056979441111773;
	distCoeffs.at<double>(2, 0) = -2.656378643258185e-04;
	distCoeffs.at<double>(3, 0) = 0.001513705053758;
	distCoeffs.at<double>(4, 0) = 0;
	
	Mat map1, map2;
	Size imRawSize(imRawW, imRawH);
	initUndistortRectifyMap(cameraMatrix,
		distCoeffs,
		Mat(),
		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imRawSize, 1, imRawSize, 0),
		imRawSize,
		CV_16SC2,
		map1,
		map2);
	
	//剪切平板位置图像
	float plateRegionHeight = 0.55;//, plateRegionWidth = 0.52;
	float plateRegionOffH = -0.033, plateRegionOffW = 0.03;
	Rect plateRegion(
		int(imRawW*(0.5 + plateRegionOffW) - imRawH*plateRegionHeight / 2),
		int(imRawH*(0.5 + plateRegionOffH - plateRegionHeight / 2)),
		imRawH*plateRegionHeight,
		imRawH*plateRegionHeight);
	imRawW = plateRegion.width;
	imRawH = plateRegion.height;
	
	//平板小球尺寸信息
	const float maxX = 600, maxY = 600;
	const float camHeight = 430;
	const float ballArea = 28;
	const float ballAreaLow = ballArea * 0.2;
	const float ballAreaHigh = ballArea * 1.5;
	
	
#ifdef SOCKET_SEND_IMAGE
	//发送图像，用于测试
	Mat imSend;
#endif // SOCKET_SEND_IMAGE
	

	
	///预处理
	Mat imProcess;
	const int structElementSize = 1;
	Mat element = getStructuringElement(MORPH_ELLIPSE,  
		Size(2*structElementSize + 1, 2*structElementSize + 1),  
		Point(structElementSize, structElementSize));
	
	
	///初始化
	//初始化连接
	if (!cam.open())
		return 1;
	
	//初始化串口
	UartNum<float> uart;
	uart.begin();
	
	//初始化阈值，固定二值化阈值，减轻后面计算负担
	const int structElementSizePre = 20;
	Mat elementPre = getStructuringElement(MORPH_ELLIPSE,  
		Size(2*structElementSize + 1, 2*structElementSize + 1),  
		Point(structElementSize, structElementSize));
	double threshBinary = 0;
	for (int i = 0; i < 60; i++)
	{
		cam.grab();
		cam.retrieve(imRaw);
		if (imRaw.empty())
		{
			cout << "imRaw为空！！！" << endl;
			return 1;
		}
		remap(imRaw, imRaw, map1, map2, INTER_LINEAR);//INTER_NEAREST
		imRaw = imRaw(plateRegion);
		morphologyEx(imRaw, imRaw, CV_MOP_DILATE, elementPre);
		
		double minBrightness;
		minMaxLoc(imRaw, &minBrightness, NULL, NULL, NULL);
		threshBinary += minBrightness; 
		
#ifdef SOCKET_SEND_IMAGE
		//发送图像，用于测试
//		threshold(imTrans, imSend, threshBinary, 255, CV_THRESH_BINARY);
		socketMat.transmit(imRaw, 80);
#endif // SOCKET_SEND_IMAGE
	}
	threshBinary /= 60;
	threshBinary -= 10;

	double timeStart = 0, timeEnd = 0;

	
	
	while (1)
	{
		cam.grab();
		cam.retrieve(imRaw);

		if (imRaw.empty())
		{
			cout << "imRaw为空！！！" << endl;
			return 1;
		}
		
		remap(imRaw, imRaw, map1, map2, INTER_LINEAR);//INTER_NEAREST
		imRaw = imRaw(plateRegion);
			
		
		/// 小球定位算法开始
		float pos[2];
		morphologyEx(imRaw, imProcess, CV_MOP_ERODE, element);
//		resize(imRaw, imThresh, Size(0, 0), resizeThresh, resizeThresh, INTER_NEAREST);
		threshold(imProcess, imProcess, threshBinary, 255, CV_THRESH_BINARY_INV);
//		threshold(imThresh, imThresh, 0, 255, CV_THRESH_OTSU);
//		morphologyEx(imRaw, imProcess, CV_MOP_TOPHAT, element);
//		medianBlur(imProcess, imProcess, 9);
//		GaussianBlur(imRaw, imProcess, 
//			Size(int(0.01*imRawH) * 2 + 1, int(0.01*imRawW) * 2 + 1), 
//			int(0.01*imRawW) * 2 + 1, int(0.01*imRawW) * 2 + 1);
		//以小球半径的两倍为窗口长度
		
		//计算二值图形的矩特征
		Moments mu = moments(imProcess, true);
		Point2f ballPoint(mu.m10 / mu.m00, mu.m01 / mu.m00);
		pos[0] = ballPoint.x;
		pos[1] = ballPoint.y;
		//通过面积判断是否有干扰
		float darkArea = mu.m00;
		if (darkArea > ballAreaHigh || darkArea < ballAreaLow)
		{
			pos[0] = 0.0 / 0.0;
			pos[1] = 0.0 / 0.0;
		}
	
		
////		轮廓提取
//		vector<vector<Point> > contours;
//		findContours(imProcess, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		
//		Point ballPoint;
//		double minBrightness;
//		minMaxLoc(imProcess, &minBrightness, NULL, &ballPoint, NULL);
//		if (minBrightness < 30)
//		{
//			pos[0] = ballPoint.x;
//			pos[1] = ballPoint.y;
//		}
//		else
//		{
//			pos[0] = -1;
//			pos[1] = -1;
//		}//如果最小亮度过高，认为小球掉落

	
		//标准化坐标单位mm
		pos[0] = pos[0]*maxX / imProcess.rows;
		pos[1] = pos[1]*maxY / imProcess.cols;
		pos[0] = 430*tan(-0.6092 + pos[0]  * 0.6092 / 300) + maxX / 2;
		pos[1] = 430*tan(-0.6092 + pos[1]  * 0.6092 / 300) + maxY / 2;
		
		
#ifdef STDIO_DEBUG
		timeEnd = (double)getTickCount();
		//计算算法帧率
		cout << "fps: " << 1.0 / (timeEnd - timeStart)*(double)getTickFrequency()
				<< "\t" << pos[0] << " of " << maxX 
				<< "\t" << pos[1] << " of " << maxX
				<< "\t" << "thres: " << threshBinary 
				<< "\t" << "darkArea:" << darkArea
				<< endl;
		timeStart = timeEnd;
#endif // STDIO_DEBUG

		uart.sendNum(pos, 2);

		///小球定位算法结束
		
		
		
#ifdef SOCKET_SEND_IMAGE
		//发送图像，用于测试
		resize(imProcess, imSend, Size(0, 0), 1, 1, INTER_NEAREST);
//		threshold(imTrans, imSend, threshBinary, 255, CV_THRESH_BINARY);
		socketMat.transmit(imSend, 80);
#endif // SOCKET_SEND_IMAGE
		
		//判断单片机发来的关机指令或按键发来的关机指令
		int fd = uart.getFd();
		if (int num = serialDataAvail(fd) > 0)
		{
			char c;
			c = serialGetchar(fd);
			for (int i = 1; i < num; i++)
			{
				serialGetchar(fd);
			}
			if (c == 's')
			{
				system("sudo shutdown -h now");
			}
//			else if (c == 'k')
//			{
//				system("sudo killall OpenCVCameraDemo");
//			}
		}
		
		
		
	}
#ifdef SOCKET_SEND_IMAGE
	socketMat.disconnect();
#endif // SOCKET_SEND_IMAGE
	cam.release();
	return 0;
}




