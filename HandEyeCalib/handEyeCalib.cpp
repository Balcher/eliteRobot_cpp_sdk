#include "handEyeCalib.hpp"

Eigen::Matrix3d HandEyeCalib::eulerAnglesToRotationMatrix(double rx, double ry, double rz)
{
	// ��X�����ת����
	Eigen::Matrix3d Rx;
	Rx << 1, 0, 0,
		0, cos(rx), -sin(rx),
		0, sin(rx), cos(rx);

    // ��y�����ת����
    Eigen::Matrix3d Ry;
    Ry << cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry);

    // ��z�����ת����
    Eigen::Matrix3d Rz;
    Rz << cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1;

    // ����z����ת������y����ת�������x����ת
    Eigen::Matrix3d R = Rz * Ry * Rx;

    return R;
}

Eigen::Matrix4d HandEyeCalib::poseToHomogeneousMatrix(const std::vector<double>& pose)
{
    double x = pose[0];
    double y = pose[1];
    double z = pose[2];
    double rx = pose[3];
    double ry = pose[4];
    double rz = pose[5];

    // ������ת����
    Eigen::Matrix3d R = eulerAnglesToRotationMatrix(rx, ry, rz);

    // ������α任����
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H.block<3, 3>(0, 0) = R;    // ����ת����ֵ����ξ���
    H.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z); // ����ƽ������

    return H;
}

void HandEyeCalib::saveMatrixToCSV(const std::vector<Eigen::Matrix4d>& matrixes, const std::string& fileName)
{
    // �����������Ƿ�Ϊ��
    if (matrixes.empty())
    {
        std::cerr << "The list of matrixes is empty" << std::endl;
        return;
    }

    // ��ȡ��������������
    int rows = matrixes[0].rows();
    int cols = matrixes[0].cols();
    int numMatrixes = matrixes.size();

    // ��ʼ���ϲ�����
    Eigen::MatrixXd combineMatrix = Eigen::MatrixXd::Zero(rows, cols * numMatrixes);

    // ������������ƴ�ӵ��ϲ�����
    for (int i = 0; i < numMatrixes; ++i)
    {
        if (matrixes[i].cols() != cols || matrixes[i].rows() != rows)
        {
            std::cerr << "All Matrixes must have the same dimensions" << std::endl;
            return;
        }
        combineMatrix.block(0, i * cols, rows, cols) = matrixes[i];
    }

    // ��CSV�ļ�����д��
    std::ofstream csvFile(fileName);
    if (!csvFile.is_open())
    {
        std::cerr << "Failed to open the file : " << fileName << std::endl;
        return;
    }

    // д�����ݵ�CSV�ļ�
    for (int r = 0; r < combineMatrix.rows(); ++r)
    {
        for (int c = 0; c < combineMatrix.cols(); ++c)
        {
            csvFile << combineMatrix(r, c);
            if (c < combineMatrix.cols() - 1)
            {
                csvFile << ",";
            }
        }
        csvFile << "\n";
    }

    csvFile.close();
    std::cout << "Matrixes saved successfully to " << fileName << std::endl;
}

void HandEyeCalib::poseSaveCSV(const std::string& poseFilePath, const std::string& csvFilePath)
{
    std::ifstream inFile(poseFilePath);
    if (!inFile.is_open())
    {
        std::cerr << "Failed to open the file : " << poseFilePath << std::endl;
        return;
    }

    std::vector<double> lines;
    std::string line;

    // ��ȡ�ļ��е���������
    while (std::getline(inFile, line))
    {
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ','))
        {
            lines.push_back(std::stod(value));    // ���ַ���ת��Ϊ double
        }
    }

    inFile.close();

    // ��6��ֵת��Ϊ��α任����
    std::vector<Eigen::Matrix4d> matrixes;
    for (size_t i = 0; i < lines.size(); i += 6)
    {
        std::vector<double> pose(lines.begin() + i, lines.begin() + i + 6);
        Eigen::Matrix4d matrix = poseToHomogeneousMatrix(pose);
        matrixes.push_back(matrix);
    }

    // ������α任����csv�ļ�
    saveMatrixToCSV(matrixes, csvFilePath);

}

bool HandEyeCalib::is_imageFile(const std::filesystem::path& file)
{
    std::vector<std::string> image_extensions = { ".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tiff", ".webp" };
    std::string extension = file.extension().string();

    // תΪСд�������Сд���е�����
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

    for (const auto& ext : image_extensions)
    {
        if (extension == ext) {
            return true;
        }
    }

    return false;
}

int HandEyeCalib::extractNumberFromFileNames(const std::string& filename)
{
    std::string num_str;
    for (char ch : filename)
    {
        if (isdigit(ch)) {
            num_str += ch;
        }
    }

    return num_str.empty() ? -1 : std::stoi(num_str);    // ���û�����֣����� -1
}

void HandEyeCalib::readFileNameFromFolder(const std::string& folderPath, std::vector<std::string>& imgFileNames)
{
    std::vector<std::string> imageNames;
    for (const auto& entry : std::filesystem::directory_iterator(folderPath))
    {
        if (std::filesystem::is_regular_file(entry.status()) && is_imageFile(entry.path()))
        {
            // ������ļ�����ͼƬ�ļ�����ӵ��ļ����б���
            imageNames.push_back(entry.path().filename().string());
        }
    }

    // �����ļ����е����ֲ�������
    std::sort(imageNames.begin(), imageNames.end(),
        [](const std::string& a, const std::string& b) {
            return extractNumberFromFileNames(a) < extractNumberFromFileNames(b);
        });

    std::string imageFullPath;
    for (int i = 0; i < imageNames.size(); i++)
    {
        imageFullPath = folderPath + "/" + imageNames[i];
        imgFileNames.push_back(imageFullPath);
    }

    //for (auto img : imgFileNames)
    //    std::cout << "imgPath = " << img << std::endl;

}

void HandEyeCalib::CalibInit(const std::string& imgfolderPath, const std::string& poseFilePath)
{
    
    // STEP 1. ��ȡͼƬ���ڴ���
    // ��ȡͼƬ��·���б�
    std::vector<std::string> imgFileNames;
    readFileNameFromFolder(imgfolderPath, imgFileNames);

    // ��ȡͼƬ�浽�ڴ���
    cv::Mat img;
    for (int i = 0; i < imgFileNames.size(); i++)
    {
        img = cv::imread(imgFileNames[i]);
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        calibImages_.push_back(img);
    }

    // STEP 2. ��ȡλ�˵��ڴ���
    std::ifstream inFile(poseFilePath);
    if (!inFile.is_open())
    {
        std::cerr << "Failed to open the file : " << poseFilePath << std::endl;
        return;
    }
    std::vector<double> lines;
    std::string line;

    // ��ȡ�ļ��е���������
    while (std::getline(inFile, line))
    {
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ','))
        {
            lines.push_back(std::stod(value));    // ���ַ���ת��Ϊ double
        }
    }

    inFile.close();

    // ��6��ֵת��Ϊ��α任����, ŷ������ ZYXת����ʽ
    for (size_t i = 0; i < lines.size(); i += 6)
    {
        std::vector<double> pose(lines.begin() + i, lines.begin() + i + 6);
        Eigen::Matrix4d matrix = poseToHomogeneousMatrix(pose);
        poses_.push_back(matrix);
    }

}

void HandEyeCalib::cameraCalib()
{
    std::cout << "========================" << std::endl;
    std::cout << "       ����ڲα궨     " << std::endl;
    std::cout << "========================" << std::endl;
    std::cout << "�궨���г��߷����Ӧ�Ľǵ����Ϊ �� " << cornerPointLong_ << std::endl;
    std::cout << "�궨���ж̱߷����Ӧ�Ľǵ����Ϊ �� " << cornerPointShort_ << std::endl;
    std::cout << "�궨���з�����ʵ�ߴ�Ϊ �� " << cornerPointSize_ << " m." << std::endl;

    // ����Ѱ�������ؽǵ�Ĳ���
    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001); // ���������� 30���������0.001

    // ��ȡ�궨��ǵ��λ��(3ά�ǵ�)
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < cornerPointShort_; ++i)
    {
        for (int j = 0; j < cornerPointLong_; ++j)
        {
            objp.emplace_back(j * cornerPointSize_, i * cornerPointSize_, 0);
        }
    }

    std::vector<std::vector<cv::Point3f>> obj_points;    // �洢����ͼ���е�3D��
    std::vector<std::vector<cv::Point2f>> img_points;    // �洢����ͼ���е�2D��

    // Ѱ��������2d��
    for (int i = 0; i < calibImages_.size(); ++i)
    {
        // ���ǵ�
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(calibImages_[i], cv::Size(cornerPointLong_, cornerPointShort_), corners);

        if (found) {
            obj_points.emplace_back(objp);

            // Ѱ�������ؽǵ�
            cv::cornerSubPix(calibImages_[i], corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

            img_points.emplace_back(corners);
        }
    }

    // ����궨
    double rms = cv::calibrateCamera(obj_points, img_points, calibImages_[0].size(),
        cameraIntrinsicMatrix_, cameraDistortionMatrix_, rvecs_, tvecs_);  // ����������ϵ���������ϵ�ı任����
    // ����궨���
    std::cout << "RMS Error: " << rms << std::endl;
    std::cout << "Camera Matrix:\n" << cameraIntrinsicMatrix_ << std::endl;
    std::cout << "Distortion Coefficients:\n" << cameraDistortionMatrix_ << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
}

Eigen::Matrix4d HandEyeCalib::computeEyeInHandT(cv::HandEyeCalibrationMethod method)
{
    // STEP 1������ڲα궨
    cameraCalib();


    // STEP 2���������ϱ궨
    std::cout << "========================" << std::endl;
    std::cout << "       �������ϱ궨     " << std::endl;
    std::cout << "========================" << std::endl;

    // ��������������ĩ��ִ����֮��ı任��ϵ
    cv::Mat R_cam2gripper, t_cam2gripper;

    // ת����cv::Mat��ʽ����ת�����ƽ������
    std::vector<cv::Mat> R_tool, t_tool;
    for (const auto& pose : poses_)
    {
        // ��ȡ��ת����
        Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);

        // ��ȡƽ������
        Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

        // ת����mat��ʽ
        // �����Ҫת��һ�µ�ԭ������ΪEigen��Mat������˳��һ����Eigen��������Mat�������� 
        /*
        * Ҳ����˵ Eigen �����ڴ������˳���� :
        * [0] = rotation[0][0]
        * [1] = rotation[1][0]
        * [2] = rotation[2][0]
        * [3] = rotation[3][0]
        * [4] = rotation[0][1]
        * ...
        * 
        * �������Ϊ��ֱ��ʹ��data(),�Ƚ�rotation�ȷ�תһ��
        */ 
        Eigen::Matrix3d rotationTemp = rotation.transpose();
        cv::Mat rotationMat(3, 3, CV_64F, rotationTemp.data());
        cv::Mat translationMat(3, 1, CV_64F, translation.data());

        R_tool.push_back(rotationMat.clone());
        t_tool.push_back(translationMat.clone());


    }

    cv::calibrateHandEye(R_tool, t_tool, rvecs_, tvecs_,
        R_cam2gripper, t_cam2gripper, method);

    // ����ƽ����ת�����λ������
    cv::Mat final_R = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat final_t = cv::Mat::zeros(3, 1, CV_64F);

    for (size_t i = 0; i < R_tool.size(); ++i) {
        final_R += R_tool[i];
        final_t += t_tool[i];
    }

    // ��ӡ���ս��
    std::cout << "R_cam2gripper (ƽ����ת����):\n" << R_cam2gripper << std::endl;
    std::cout << "t_cam2gripper (ƽ��ƽ������):\n" << t_cam2gripper << std::endl;

    // ��cv::Mat ת��Ϊ Eigen::Matrix4d
    Eigen::Matrix4d eyeInHandT = Eigen::Matrix4d::Identity();    // ��ʼ��Ϊ��λ����

    // ����ת��������
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            eyeInHandT(i, j) = R_cam2gripper.at<double>(i, j);
        }
    }

    // ��ƽ����������
    for (int i = 0; i < 3; ++i)
    {
        eyeInHandT(i, 3) = t_cam2gripper.at<double>(i, 0);
    }

    return eyeInHandT;
}

Eigen::Matrix4d HandEyeCalib::computeEyeToHandT(cv::HandEyeCalibrationMethod method)
{
    // STEP 1������ڲα궨
    cameraCalib();

    // STEP 2����������궨
    std::cout << "========================" << std::endl;
    std::cout << "       ��������궨     " << std::endl;
    std::cout << "========================" << std::endl;


    // ����������е�ۻ���֮��ı任��ϵ
    cv::Mat R_cam2base, t_cam2base;

    // ת����cv::Mat��ʽ����ת�����ƽ������
    std::vector<cv::Mat> R_tool, t_tool;
    for (const auto& pose : poses_)
    {
        // ��Ϊ�õ���pose�� ��е��ĩ�˵���е�ۻ����ı任����
        // Ȼ�����ڼ�������������⣬������Ҫ���� ��е�ۻ�������е��ĩ�˵ı任����
        Eigen::Matrix4d baseToEnd = pose.inverse();

        // ��ȡ��ת����
        Eigen::Matrix3d rotation = baseToEnd.block<3, 3>(0, 0);

        // ��ȡƽ������
        Eigen::Vector3d translation = baseToEnd.block<3, 1>(0, 3);

        // ת����mat��ʽ
        // �����Ҫת��һ�µ�ԭ������ΪEigen��Mat������˳��һ����Eigen��������Mat�������� 
        /*
        * Ҳ����˵ Eigen �����ڴ������˳���� :
        * [0] = rotation[0][0]
        * [1] = rotation[1][0]
        * [2] = rotation[2][0]
        * [3] = rotation[3][0]
        * [4] = rotation[0][1]
        * ...
        *
        * �������Ϊ��ֱ��ʹ��data(),�Ƚ�rotation�ȷ�תһ��
        */
        Eigen::Matrix3d rotationTemp = rotation.transpose();
        cv::Mat rotationMat(3, 3, CV_64F, rotationTemp.data());
        cv::Mat translationMat(3, 1, CV_64F, translation.data());

        R_tool.push_back(rotationMat.clone());
        t_tool.push_back(translationMat.clone());


    }

    cv::calibrateHandEye(R_tool, t_tool, rvecs_, tvecs_,
        R_cam2base, t_cam2base, method);

    // ����ƽ����ת�����λ������
    cv::Mat final_R = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat final_t = cv::Mat::zeros(3, 1, CV_64F);

    for (size_t i = 0; i < R_tool.size(); ++i) {
        final_R += R_tool[i];
        final_t += t_tool[i];
    }

    // ��ӡ���ս��
    std::cout << "R_cam2base (ƽ����ת����):\n" << R_cam2base << std::endl;
    std::cout << "t_cam2base (ƽ��ƽ������):\n" << t_cam2base << std::endl;

    // ��cv::Mat ת��Ϊ Eigen::Matrix4d
    Eigen::Matrix4d eyeToHandT = Eigen::Matrix4d::Identity();    // ��ʼ��Ϊ��λ����

    // ����ת��������
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            eyeToHandT(i, j) = R_cam2base.at<double>(i, j);
        }
    }

    // ��ƽ����������
    for (int i = 0; i < 3; ++i)
    {
        eyeToHandT(i, 3) = t_cam2base.at<double>(i, 0);
    }

    return eyeToHandT;
}





