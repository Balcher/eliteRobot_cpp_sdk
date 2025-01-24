#include <iostream>
#include <opencv2/opencv.hpp>
#include <HandEyeCalib/handEyeCalib.hpp>



int main()
{
	std::cout << "cv version = " << CV_VERSION << std::endl;
	std::string Fileroot = "D:\\gitResository\\eliteRobot_cpp_sdk\\HandEyeCalib\\";

	//// ��λ�˱��浽csv�ļ��в��ԣ�ʵ��û��ʲô�ã������ã�
	std::string filePath = Fileroot + "images\\poses.txt";
	std::string csvFilePath = Fileroot + "csvFile.csv";
	HandEyeCalib* myHandEyeCalib = new HandEyeCalib(8, 5, 0.027);
	myHandEyeCalib->poseSaveCSV(filePath, csvFilePath);

	// ���ļ��ж�ȡͼƬ����
	std::vector<std::string> imagefiles;
	std::string imagefolder = Fileroot + "images";
	std::cout << "imagefolder = " << imagefolder << std::endl;
	myHandEyeCalib->readFileNameFromFolder(imagefolder, imagefiles);

	// �궨��ʼ��
	std::string imageFolder = Fileroot + "images";
	std::string poseFile = Fileroot + "images\\poses.txt";
	myHandEyeCalib->CalibInit(imageFolder, poseFile);

	Eigen::Matrix4d eyInoHandT = myHandEyeCalib->computeEyeInHandT(); // �������ϱ궨
	std::cout << "eyInoHandT = \n" << eyInoHandT << std::endl;

	Eigen::Matrix4d eyeToHandT = myHandEyeCalib->computeEyeToHandT(); // ��������궨
	std::cout << "eyeToHandT = \n" << eyeToHandT << std::endl;

}
