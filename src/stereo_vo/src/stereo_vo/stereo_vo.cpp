#include "../../include/stereo_vo/stereo_vo.h"

namespace stereo_vo
{
    StereoVO::StereoVO()
    {
    }

    StereoVO::~StereoVO()
    {
    }

    void StereoVO::init()
    {
        assert(_params_init_flag);
        this->frontend_ = Frontend::Ptr(new Frontend(300, 100));
        this->backend_ = Backend::Ptr(new Backend);
        this->map_ = Map::Ptr(new Map);
        cv::stereoRectify(this->params.camera_matrix_left, this->params.distortion_coefficients_left, this->params.camera_matrix_right, this->params.distortion_coefficients_right,
                          cv::Size(this->_frame_width / 2, this->_frame_height), this->params.rotation_of_camera_right, this->params.translation_of_camera_right, this->params.Rl, this->params.Rr, this->params.Pl, this->params.Pr,
                          this->params.Q, cv::CALIB_ZERO_DISPARITY, 0);
        cv::initUndistortRectifyMap(this->params.camera_matrix_left, this->params.distortion_coefficients_left, this->params.Rl, this->params.Pl, cv::Size(this->_frame_width / 2, this->_frame_height), CV_16SC2, undistmap1l, undistmap2l);
        cv::initUndistortRectifyMap(this->params.camera_matrix_right, this->params.distortion_coefficients_right, this->params.Rr, this->params.Pr, cv::Size(this->_frame_width / 2, this->_frame_height), CV_16SC2, undistmap1r, undistmap2r);
        this->params.rotation_of_camera_right = this->params.Rr * this->params.rotation_of_camera_right * this->params.Rl.t();
        this->params.translation_of_camera_right = this->params.Rr * this->params.translation_of_camera_right;
        this->params.baseline = this->params.translation_of_camera_right.at<double>(cv::Point2i(0, 0));
        Mat33 K_1, K_2;
        K_1 << this->params.Pl.at<double>(0, 0), this->params.Pl.at<double>(0, 1), this->params.Pl.at<double>(0, 2),
            this->params.Pl.at<double>(1, 0), this->params.Pl.at<double>(1, 1), this->params.Pl.at<double>(1, 2),
            this->params.Pl.at<double>(2, 0), this->params.Pl.at<double>(2, 1), this->params.Pl.at<double>(2, 2);
        K_2 << this->params.Pr.at<double>(0, 0), this->params.Pr.at<double>(0, 1), this->params.Pr.at<double>(0, 2),
            this->params.Pr.at<double>(1, 0), this->params.Pr.at<double>(1, 1), this->params.Pr.at<double>(1, 2),
            this->params.Pr.at<double>(2, 0), this->params.Pr.at<double>(2, 1), this->params.Pr.at<double>(2, 2);
        Vec3 t_1, t_2;
        t_1 << 0.0, 0.0, 0.0;
        t_2 << this->params.translation_of_camera_right.at<double>(0, 0), this->params.translation_of_camera_right.at<double>(1, 0), this->params.translation_of_camera_right.at<double>(2, 0);
        Camera::Ptr new_camera_l(new Camera(K_1(0, 0), K_1(1, 1), K_1(0, 2), K_1(1, 2),
                                            t_1.norm(), SE3(SO3(), t_1), undistmap1l, undistmap2l));
        Camera::Ptr new_camera_r(new Camera(K_2(0, 0), K_2(1, 1), K_2(0, 2), K_2(1, 2),
                                            t_2.norm(), SE3(SO3(), t_2), undistmap1r, undistmap2r));

        double focal_length = this->params.camera_matrix_left.at<double>(0, 0);
        double max_depth = (this->params.baseline * focal_length) / (this->params.baseline / (double(this->_frame_width) / 2.)) * 0.01;
        this->frontend_->SetMaxDepth(max_depth);
        this->frontend_->SetCameras(new_camera_l, new_camera_r);
        this->frontend_->SetBackend(this->backend_);
        this->frontend_->SetMap(this->map_);

        this->backend_->SetCameras(new_camera_l, new_camera_r);
        this->backend_->SetMap(this->map_);
    }

    void StereoVO::stop()
    {
        this->backend_->Stop();
    }

    bool StereoVO::read_param(const std::string &path, const int frame_width, const int frame_height)
    {
        this->_frame_width = frame_width;
        this->_frame_height = frame_height;
        if (access(path.c_str(), F_OK) != 0)
            return false;
        cv::FileStorage fs;
        try
        {
            fs = cv::FileStorage(path, cv::FileStorage::READ);
        }
        catch (const std::exception &e)
        {
            if (_ifShowErr)
                std::cerr << e.what() << '\n';
            return false;
        }
        cv::Mat frame_size;
        fs["camera_matrix_left"] >> this->params.camera_matrix_left;
        fs["camera_matrix_right"] >> this->params.camera_matrix_right;
        fs["distortion_coefficients_left"] >> this->params.distortion_coefficients_left;
        fs["distortion_coefficients_right"] >> this->params.distortion_coefficients_right;
        fs["translation_of_camera_right"] >> this->params.translation_of_camera_right;
        fs["rotation_of_camera_right"] >> this->params.rotation_of_camera_right;
        fs["frame_size"] >> frame_size;
        this->_frame_width = frame_size.at<int>(0, 0);
        this->_frame_height = frame_size.at<int>(0, 1);
        this->_params_init_flag = true;
        return true;
    }

    bool StereoVO::_is_params_inited()
    {
        return _params_init_flag;
    }

    bool StereoVO::addFrame(cv::Mat &img_l, cv::Mat &img_r)
    {
        if (img_l.empty() || img_r.empty())
        {
            return false;
        }
        auto new_frame = Frame::CreateFrame();
        new_frame->left_img_ = img_l;
        new_frame->right_img_ = img_r;
        this->frontend_->AddFrame(new_frame);
        return true;
    }

    bool StereoVO::getPose(Eigen::Matrix3d &rotation_matrix, Eigen::Vector3d &translation_vector)
    {
        Eigen::Matrix4d pose;
        if (this->frontend_->GetPose(pose))
        {
            rotation_matrix << pose(0, 0), pose(0, 1), pose(0, 2),
                pose(1, 0), pose(1, 1), pose(1, 2),
                pose(2, 0), pose(2, 1), pose(2, 2);
            translation_vector << pose(0, 3), pose(1, 3), pose(2, 3);
            translation_vector = rotation_matrix.inverse() * translation_vector;
            return true;
        }
        return false;
    }

    void StereoVO::GetMapPoints(Map::LandmarksType &points)
    {
        points = this->map_->GetAllMapPoints();
    }
} // namespace stereo_vo
