#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>

class HandEyeCalib
{

public:

	explicit HandEyeCalib(int cornerPointLong, int cornerPointShort, float cornerPointSize) : 
		cornerPointLong_(cornerPointLong), cornerPointShort_(cornerPointShort), cornerPointSize_(cornerPointSize) {}
	~HandEyeCalib() {}

	HandEyeCalib() = delete;

	////////////////////////////////////////////////////////////
	///                   �ļ���ȡ��غ���
	///////////////////////////////////////////////////////////
 
	/// @brief ��ŷ����ת������ת����ZYX��ʽ��
	/// @param rx ��x����ת�Ƕ�
	/// @param ry ��y����ת�Ƕ�
	/// @param rz ��z����ת�Ƕ�
	/// @return ��ת����
	Eigen::Matrix3d eulerAnglesToRotationMatrix(double rx, double ry, double rz);

	/// @brief ��������ʽ��poseת������ξ���
	/// @param pose ������ʽ��pose
	/// @return ��ξ���
	Eigen::Matrix4d poseToHomogeneousMatrix(const std::vector<double>& pose);

	/// @brief ������洢��CSV�ļ���
	/// @param matrixes ��ξ�������
	/// @param fileName �洢��csv�ļ�����
	void saveMatrixToCSV(const std::vector<Eigen::Matrix4d>& matrixes, const std::string& fileName);

	/// @brief ��ȡtxt�ļ��е�pose��Ȼ��洢��csv��
	/// @param poseFilePath pose�ļ���
	/// @param csvFilePath  �洢��csv�ļ�·��������
	void poseSaveCSV(const std::string& poseFilePath, const std::string& csvFilePath);

	/// @brief �ж��ļ��Ƿ���ͼƬ��ʽ
	/// @param filePath ͼƬ�ļ�
	/// @return true��ͼƬ��ʽ��false������ͼƬ��ʽ
	static bool is_imageFile(const std::filesystem::path& file);

	/// @brief ��ȡ�ļ����е����ֲ���
	/// @param filename �ļ���
	static int extractNumberFromFileNames(const std::string& filename);

	/// @brief ���ļ����ж�ȡͼƬ����
	/// @param foldPath �ļ���·��
	/// @param imgFileNames ͼƬ����
	static void readFileNameFromFolder(const std::string& folderPath, std::vector<std::string>& imgFileNames);

	////////////////////////////////////////////////////////////
	///                       �궨���
	///////////////////////////////////////////////////////////
	
	int cornerPointLong_;                 ///< �궨��ǵ�����������
	int cornerPointShort_;                ///< �궨��ǵ��������̱�
	float cornerPointSize_;               ///< �궨�巽����ʵ�ߴ磬m

	cv::Mat cameraIntrinsicMatrix_;       ///< ����ڲξ���
	cv::Mat cameraDistortionMatrix_;      ///< ��������������[k1, k2, p1, p2, k3]
	std::vector<cv::Mat> rvecs_, tvecs_;  ///< ����궨�����еõ�����ת�����ƽ������,����������ϵ���������ϵ�ı任����
	
	std::vector<cv::Mat> calibImages_;    ///< ���۱궨��ͼƬ����
	std::vector<Eigen::Matrix4d> poses_; ///< ��ǰ��е��ĩ�˵�������ת������

	/// @brief ��ʼ����������ȡͼƬ��λ��
	/// @param imgfolderPath   ͼƬ�ļ���·��
	/// @param poseFilePath    λ���ļ�·��
	void CalibInit(const std::string& imgfolderPath, const std::string& poseFilePath);

	void cameraCalib();

	/// @brief �������ϱ궨
	/// @brief method ���۱궨�ķ���,Ĭ��ʹ��CALIB_HAND_EYE_TSAI
	/// @return �������ϵ����е��ĩ�˵ı任����
	Eigen::Matrix4d computeEyeInHandT(cv::HandEyeCalibrationMethod method = cv::CALIB_HAND_EYE_TSAI);

	/// @brief ��������궨
	/// @return �������ϵ����е�ۻ����ı任����
	Eigen::Matrix4d computeEyeToHandT(cv::HandEyeCalibrationMethod method = cv::CALIB_HAND_EYE_TSAI);


};
