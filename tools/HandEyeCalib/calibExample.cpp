#include <iostream>
#include <opencv2/opencv.hpp>
#include "handEyeCalib.hpp"



int main()
{
	std::cout << "cv version = " << CV_VERSION << std::endl;
	std::string Fileroot = "../calib4.7/";

	std::string filePath = Fileroot + "poses.txt";
	std::string csvFilePath = Fileroot + "csvFile.csv";
	HandEyeCalib* myHandEyeCalib = new HandEyeCalib(9, 6, 0.025);
	// myHandEyeCalib->poseSaveCSV(filePath, csvFilePath);  // 将位姿保存到csv文件中测试（实际没有什么用，测试用）

	// 从文件中读取图片测试
	std::vector<std::string> imagefiles;
	std::string imagefolder = Fileroot;
	std::cout << "imagefolder = " << imagefolder << std::endl;
	myHandEyeCalib->readFileNameFromFolder(imagefolder, imagefiles);

	// 标定初始化
	std::string imageFolder = Fileroot;
	std::string poseFile = Fileroot + "poses.txt";
	myHandEyeCalib->CalibInit(imageFolder, poseFile);

	Eigen::Matrix4d eyeInHandT = myHandEyeCalib->computeEyeInHandT(); // 眼在手上标定
	std::cout << "eyeInHandT = \n" << eyeInHandT << std::endl;

	// 使用眼在手外标定只要将目录中的图片更换成眼在手外采集的图片，然后进行标定
	//Eigen::Matrix4d eyeToHandT = myHandEyeCalib->computeEyeToHandT(); // 眼在手外标定
	//std::cout << "eyeToHandT = \n" << eyeToHandT << std::endl;

}
