#include "handEyeCalib.hpp"

Eigen::Matrix3d HandEyeCalib::eulerAnglesToRotationMatrix(double rx, double ry, double rz)
{
	// 绕X轴的旋转矩阵
	Eigen::Matrix3d Rx;
	Rx << 1, 0, 0,
		0, cos(rx), -sin(rx),
		0, sin(rx), cos(rx);

    // 绕y轴的旋转矩阵
    Eigen::Matrix3d Ry;
    Ry << cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry);

    // 绕z轴的旋转矩阵
    Eigen::Matrix3d Rz;
    Rz << cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1;

    // 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转
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

    // 计算旋转矩阵
    Eigen::Matrix3d R = eulerAnglesToRotationMatrix(rx, ry, rz);

    // 创建齐次变换矩阵
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H.block<3, 3>(0, 0) = R;    // 将旋转矩阵赋值到齐次矩阵
    H.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z); // 设置平移向量

    return H;
}

void HandEyeCalib::saveMatrixToCSV(const std::vector<Eigen::Matrix4d>& matrixes, const std::string& fileName)
{
    // 检查输入矩阵是否为空
    if (matrixes.empty())
    {
        std::cerr << "The list of matrixes is empty" << std::endl;
        return;
    }

    // 获取矩阵行数和列数
    int rows = matrixes[0].rows();
    int cols = matrixes[0].cols();
    int numMatrixes = matrixes.size();

    // 初始化合并矩阵
    Eigen::MatrixXd combineMatrix = Eigen::MatrixXd::Zero(rows, cols * numMatrixes);

    // 将输入矩阵横向拼接到合并矩阵
    for (int i = 0; i < numMatrixes; ++i)
    {
        if (matrixes[i].cols() != cols || matrixes[i].rows() != rows)
        {
            std::cerr << "All Matrixes must have the same dimensions" << std::endl;
            return;
        }
        combineMatrix.block(0, i * cols, rows, cols) = matrixes[i];
    }

    // 打开CSV文件进行写入
    std::ofstream csvFile(fileName);
    if (!csvFile.is_open())
    {
        std::cerr << "Failed to open the file : " << fileName << std::endl;
        return;
    }

    // 写入数据到CSV文件
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

    // 读取文件中的所有数据
    while (std::getline(inFile, line))
    {
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ','))
        {
            lines.push_back(std::stod(value));    // 将字符串转换为 double
        }
    }

    inFile.close();

    // 将6个值转换为齐次变换矩阵
    std::vector<Eigen::Matrix4d> matrixes;
    for (size_t i = 0; i < lines.size(); i += 6)
    {
        std::vector<double> pose(lines.begin() + i, lines.begin() + i + 6);
        Eigen::Matrix4d matrix = poseToHomogeneousMatrix(pose);
        matrixes.push_back(matrix);
    }

    // 保存齐次变换矩阵到csv文件
    saveMatrixToCSV(matrixes, csvFilePath);

}

bool HandEyeCalib::is_imageFile(const std::filesystem::path& file)
{
    std::vector<std::string> image_extensions = { ".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tiff", ".webp" };
    std::string extension = file.extension().string();

    // 转为小写，避免大小写敏感的问题
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

    return num_str.empty() ? -1 : std::stoi(num_str);    // 如果没有数字，返回 -1
}

void HandEyeCalib::readFileNameFromFolder(const std::string& folderPath, std::vector<std::string>& imgFileNames)
{
    std::vector<std::string> imageNames;
    for (const auto& entry : std::filesystem::directory_iterator(folderPath))
    {
        if (std::filesystem::is_regular_file(entry.status()) && is_imageFile(entry.path()))
        {
            // 如果是文件且是图片文件，添加到文件名列表中
            imageNames.push_back(entry.path().filename().string());
        }
    }

    // 按照文件名中的数字部分排序
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
    
    // STEP 1. 读取图片到内存中
    // 获取图片的路径列表
    std::vector<std::string> imgFileNames;
    readFileNameFromFolder(imgfolderPath, imgFileNames);

    // 读取图片存到内存中
    cv::Mat img;
    for (int i = 0; i < imgFileNames.size(); i++)
    {
        img = cv::imread(imgFileNames[i]);
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        calibImages_.push_back(img);
    }

    // STEP 2. 读取位姿到内存中
    std::ifstream inFile(poseFilePath);
    if (!inFile.is_open())
    {
        std::cerr << "Failed to open the file : " << poseFilePath << std::endl;
        return;
    }
    std::vector<double> lines;
    std::string line;

    // 读取文件中的所有数据
    while (std::getline(inFile, line))
    {
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ','))
        {
            lines.push_back(std::stod(value));    // 将字符串转换为 double
        }
    }

    inFile.close();

    // 将6个值转换为齐次变换矩阵, 欧拉角是 ZYX转换形式
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
    std::cout << "       相机内参标定     " << std::endl;
    std::cout << "========================" << std::endl;
    std::cout << "标定板中长边方向对应的角点个数为 ： " << cornerPointLong_ << std::endl;
    std::cout << "标定板中短边方向对应的角点个数为 ： " << cornerPointShort_ << std::endl;
    std::cout << "标定板中方格真实尺寸为 ： " << cornerPointSize_ << " m." << std::endl;

    // 设置寻找亚像素角点的参数
    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001); // 最大迭代次数 30或误差容限0.001

    // 获取标定板角点的位置(3维角点)
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < cornerPointShort_; ++i)
    {
        for (int j = 0; j < cornerPointLong_; ++j)
        {
            objp.emplace_back(j * cornerPointSize_, i * cornerPointSize_, 0);
        }
    }

    std::vector<std::vector<cv::Point3f>> obj_points;    // 存储所有图像中的3D点
    std::vector<std::vector<cv::Point2f>> img_points;    // 存储所有图像中的2D点

    // 寻找亚像素2d点
    for (int i = 0; i < calibImages_.size(); ++i)
    {
        // 检测角点
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(calibImages_[i], cv::Size(cornerPointLong_, cornerPointShort_), corners);

        if (found) {
            obj_points.emplace_back(objp);

            // 寻找亚像素角点
            cv::cornerSubPix(calibImages_[i], corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

            img_points.emplace_back(corners);
        }
    }

    // 相机标定
    double rms = cv::calibrateCamera(obj_points, img_points, calibImages_[0].size(),
        cameraIntrinsicMatrix_, cameraDistortionMatrix_, rvecs_, tvecs_);  // 从世界坐标系到相机坐标系的变换矩阵
    // 输出标定结果
    std::cout << "RMS Error: " << rms << std::endl;
    std::cout << "Camera Matrix:\n" << cameraIntrinsicMatrix_ << std::endl;
    std::cout << "Distortion Coefficients:\n" << cameraDistortionMatrix_ << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
}

Eigen::Matrix4d HandEyeCalib::computeEyeInHandT(cv::HandEyeCalibrationMethod method)
{
    // STEP 1：相机内参标定
    cameraCalib();


    // STEP 2：眼在手上标定
    std::cout << "========================" << std::endl;
    std::cout << "       眼在手上标定     " << std::endl;
    std::cout << "========================" << std::endl;

    // 计算相机与机器人末端执行器之间的变换关系
    cv::Mat R_cam2gripper, t_cam2gripper;

    // 转换成cv::Mat形式的旋转矩阵和平移向量
    std::vector<cv::Mat> R_tool, t_tool;
    for (const auto& pose : poses_)
    {
        // 提取旋转矩阵
        Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);

        // 提取平移向量
        Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

        // 转换成mat形式
        // 这边需要转置一下的原因是因为Eigen和Mat的排列顺序不一样，Eigen是列主序，Mat是行主序。 
        /*
        * 也就是说 Eigen 里面内存的排列顺序是 :
        * [0] = rotation[0][0]
        * [1] = rotation[1][0]
        * [2] = rotation[2][0]
        * [3] = rotation[3][0]
        * [4] = rotation[0][1]
        * ...
        * 
        * 所以这边为了直接使用data(),先将rotation先翻转一下
        */ 
        Eigen::Matrix3d rotationTemp = rotation.transpose();
        cv::Mat rotationMat(3, 3, CV_64F, rotationTemp.data());
        cv::Mat translationMat(3, 1, CV_64F, translation.data());

        R_tool.push_back(rotationMat.clone());
        t_tool.push_back(translationMat.clone());


    }

    cv::calibrateHandEye(R_tool, t_tool, rvecs_, tvecs_,
        R_cam2gripper, t_cam2gripper, method);

    // 计算平均旋转矩阵和位移向量
    cv::Mat final_R = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat final_t = cv::Mat::zeros(3, 1, CV_64F);

    for (size_t i = 0; i < R_tool.size(); ++i) {
        final_R += R_tool[i];
        final_t += t_tool[i];
    }

    // 打印最终结果
    std::cout << "R_cam2gripper (平均旋转矩阵):\n" << R_cam2gripper << std::endl;
    std::cout << "t_cam2gripper (平均平移向量):\n" << t_cam2gripper << std::endl;

    // 将cv::Mat 转换为 Eigen::Matrix4d
    Eigen::Matrix4d eyeInHandT = Eigen::Matrix4d::Identity();    // 初始化为单位矩阵

    // 将旋转矩阵填入
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            eyeInHandT(i, j) = R_cam2gripper.at<double>(i, j);
        }
    }

    // 将平移向量填入
    for (int i = 0; i < 3; ++i)
    {
        eyeInHandT(i, 3) = t_cam2gripper.at<double>(i, 0);
    }

    return eyeInHandT;
}

Eigen::Matrix4d HandEyeCalib::computeEyeToHandT(cv::HandEyeCalibrationMethod method)
{
    // STEP 1：相机内参标定
    cameraCalib();

    // STEP 2：眼在手外标定
    std::cout << "========================" << std::endl;
    std::cout << "       眼在手外标定     " << std::endl;
    std::cout << "========================" << std::endl;


    // 计算相机与机械臂基座之间的变换关系
    cv::Mat R_cam2base, t_cam2base;

    // 转换成cv::Mat形式的旋转矩阵和平移向量
    std::vector<cv::Mat> R_tool, t_tool;
    for (const auto& pose : poses_)
    {
        // 因为得到的pose是 机械臂末端到机械臂基座的变换矩阵
        // 然后现在计算的是眼在手外，所以需要计算 机械臂基座到机械臂末端的变换矩阵
        Eigen::Matrix4d baseToEnd = pose.inverse();

        // 提取旋转矩阵
        Eigen::Matrix3d rotation = baseToEnd.block<3, 3>(0, 0);

        // 提取平移向量
        Eigen::Vector3d translation = baseToEnd.block<3, 1>(0, 3);

        // 转换成mat形式
        // 这边需要转置一下的原因是因为Eigen和Mat的排列顺序不一样，Eigen是列主序，Mat是行主序。 
        /*
        * 也就是说 Eigen 里面内存的排列顺序是 :
        * [0] = rotation[0][0]
        * [1] = rotation[1][0]
        * [2] = rotation[2][0]
        * [3] = rotation[3][0]
        * [4] = rotation[0][1]
        * ...
        *
        * 所以这边为了直接使用data(),先将rotation先翻转一下
        */
        Eigen::Matrix3d rotationTemp = rotation.transpose();
        cv::Mat rotationMat(3, 3, CV_64F, rotationTemp.data());
        cv::Mat translationMat(3, 1, CV_64F, translation.data());

        R_tool.push_back(rotationMat.clone());
        t_tool.push_back(translationMat.clone());


    }

    cv::calibrateHandEye(R_tool, t_tool, rvecs_, tvecs_,
        R_cam2base, t_cam2base, method);

    // 计算平均旋转矩阵和位移向量
    cv::Mat final_R = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat final_t = cv::Mat::zeros(3, 1, CV_64F);

    for (size_t i = 0; i < R_tool.size(); ++i) {
        final_R += R_tool[i];
        final_t += t_tool[i];
    }

    // 打印最终结果
    std::cout << "R_cam2base (平均旋转矩阵):\n" << R_cam2base << std::endl;
    std::cout << "t_cam2base (平均平移向量):\n" << t_cam2base << std::endl;

    // 将cv::Mat 转换为 Eigen::Matrix4d
    Eigen::Matrix4d eyeToHandT = Eigen::Matrix4d::Identity();    // 初始化为单位矩阵

    // 将旋转矩阵填入
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            eyeToHandT(i, j) = R_cam2base.at<double>(i, j);
        }
    }

    // 将平移向量填入
    for (int i = 0; i < 3; ++i)
    {
        eyeToHandT(i, 3) = t_cam2base.at<double>(i, 0);
    }

    return eyeToHandT;
}





