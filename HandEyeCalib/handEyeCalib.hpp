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
	///                   文件读取相关函数
	///////////////////////////////////////////////////////////
 
	/// @brief 将欧拉角转换成旋转矩阵（ZYX形式）
	/// @param rx 绕x轴旋转角度
	/// @param ry 绕y轴旋转角度
	/// @param rz 绕z轴旋转角度
	/// @return 旋转矩阵
	Eigen::Matrix3d eulerAnglesToRotationMatrix(double rx, double ry, double rz);

	/// @brief 将数组形式的pose转换成齐次矩阵
	/// @param pose 数组形式的pose
	/// @return 齐次矩阵
	Eigen::Matrix4d poseToHomogeneousMatrix(const std::vector<double>& pose);

	/// @brief 将矩阵存储到CSV文件中
	/// @param matrixes 齐次矩阵数组
	/// @param fileName 存储的csv文件名称
	void saveMatrixToCSV(const std::vector<Eigen::Matrix4d>& matrixes, const std::string& fileName);

	/// @brief 读取txt文件中的pose，然后存储到csv中
	/// @param poseFilePath pose文件名
	/// @param csvFilePath  存储的csv文件路径和名称
	void poseSaveCSV(const std::string& poseFilePath, const std::string& csvFilePath);

	/// @brief 判断文件是否是图片格式
	/// @param filePath 图片文件
	/// @return true：图片格式，false：不是图片格式
	static bool is_imageFile(const std::filesystem::path& file);

	/// @brief 提取文件名中的数字部分
	/// @param filename 文件名
	static int extractNumberFromFileNames(const std::string& filename);

	/// @brief 从文件夹中读取图片名称
	/// @param foldPath 文件夹路径
	/// @param imgFileNames 图片名称
	static void readFileNameFromFolder(const std::string& folderPath, std::vector<std::string>& imgFileNames);

	////////////////////////////////////////////////////////////
	///                       标定相关
	///////////////////////////////////////////////////////////
	
	int cornerPointLong_;                 ///< 标定板角点数量，长边
	int cornerPointShort_;                ///< 标定板角点数量，短边
	float cornerPointSize_;               ///< 标定板方格真实尺寸，m

	cv::Mat cameraIntrinsicMatrix_;       ///< 相机内参矩阵
	cv::Mat cameraDistortionMatrix_;      ///< 相机畸变参数矩阵[k1, k2, p1, p2, k3]
	std::vector<cv::Mat> rvecs_, tvecs_;  ///< 相机标定过程中得到的旋转矩阵和平移向量,从世界坐标系到相机坐标系的变换矩阵
	
	std::vector<cv::Mat> calibImages_;    ///< 手眼标定的图片数据
	std::vector<Eigen::Matrix4d> poses_; ///< 当前机械臂末端到基座的转换矩阵

	/// @brief 初始化函数，读取图片和位姿
	/// @param imgfolderPath   图片文件夹路径
	/// @param poseFilePath    位姿文件路径
	void CalibInit(const std::string& imgfolderPath, const std::string& poseFilePath);

	void cameraCalib();

	/// @brief 眼在手上标定
	/// @brief method 手眼标定的方法,默认使用CALIB_HAND_EYE_TSAI
	/// @return 相机坐标系到机械臂末端的变换矩阵
	Eigen::Matrix4d computeEyeInHandT(cv::HandEyeCalibrationMethod method = cv::CALIB_HAND_EYE_TSAI);

	/// @brief 眼在手外标定
	/// @return 相机坐标系到机械臂基座的变换矩阵
	Eigen::Matrix4d computeEyeToHandT(cv::HandEyeCalibrationMethod method = cv::CALIB_HAND_EYE_TSAI);


};
