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
        assert(_params_init_flag && _is_calcTransform_flag);
        this->frontend_ = Frontend::Ptr(new Frontend(150, 50));
        this->map_ = Map::Ptr(new Map);
        Mat33 K_1, K_2;
        K_1 << this->params.camera_matrix_left.at<double>(0, 0), this->params.camera_matrix_left.at<double>(0, 1), this->params.camera_matrix_left.at<double>(0, 2),
            this->params.camera_matrix_left.at<double>(1, 0), this->params.camera_matrix_left.at<double>(1, 1), this->params.camera_matrix_left.at<double>(1, 2),
            this->params.camera_matrix_left.at<double>(2, 0), this->params.camera_matrix_left.at<double>(2, 1), this->params.camera_matrix_left.at<double>(2, 2);
        K_2 << this->params.camera_matrix_right.at<double>(0, 0), this->params.camera_matrix_right.at<double>(0, 1), this->params.camera_matrix_right.at<double>(0, 2),
            this->params.camera_matrix_right.at<double>(1, 0), this->params.camera_matrix_right.at<double>(1, 1), this->params.camera_matrix_right.at<double>(1, 2),
            this->params.camera_matrix_right.at<double>(2, 0), this->params.camera_matrix_right.at<double>(2, 1), this->params.camera_matrix_right.at<double>(2, 2);
        Vec3 t_1, t_2;
        t_1 << 0.0, 0.0, 0.0;
        t_2 << this->params.translation_of_camera_right.at<double>(0, 0), this->params.translation_of_camera_right.at<double>(1, 0), this->params.translation_of_camera_right.at<double>(2, 0);
        Camera::Ptr new_camera_l(new Camera(K_1(0, 0), K_1(1, 1), K_1(0, 2), K_1(1, 2),
                                            t_1.norm(), SE3(SO3(), t_1)));
        Camera::Ptr new_camera_r(new Camera(K_2(0, 0), K_2(1, 1), K_2(0, 2), K_2(1, 2),
                                            t_2.norm(), SE3(SO3(), t_2)));
        this->frontend_->SetCameras(new_camera_l, new_camera_r);
        this->frontend_->SetMap(this->map_);
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
        fs["camera_matrix_left"] >> this->params.camera_matrix_left;
        fs["camera_matrix_right"] >> this->params.camera_matrix_right;
        fs["distortion_coefficients_left"] >> this->params.distortion_coefficients_left;
        fs["distortion_coefficients_right"] >> this->params.distortion_coefficients_right;
        fs["translation_of_camera_right"] >> this->params.translation_of_camera_right;
        fs["rotation_of_camera_right"] >> this->params.rotation_of_camera_right;
        this->_params_init_flag = true;
        return true;
    }

    bool StereoVO::_is_params_inited()
    {
        return _params_init_flag;
    }

    void StereoVO::calcTransform()
    {
        cv::stereoRectify(this->params.camera_matrix_left, this->params.distortion_coefficients_left, this->params.camera_matrix_right, this->params.distortion_coefficients_right,
                          cv::Size(this->_frame_width / 2, this->_frame_height), this->params.rotation_of_camera_right, this->params.translation_of_camera_right, this->params.Rl, this->params.Rr, this->params.Pl, this->params.Pr,
                          this->params.Q, cv::CALIB_ZERO_DISPARITY, 0);
        initUndistortRectifyMap(this->params.camera_matrix_left, this->params.distortion_coefficients_left, this->params.Rl, this->params.Pl, cv::Size(this->_frame_width / 2, this->_frame_height), CV_16SC2, undistmap1l, undistmap2l);
        initUndistortRectifyMap(this->params.camera_matrix_right, this->params.distortion_coefficients_right, this->params.Rr, this->params.Pr, cv::Size(this->_frame_width / 2, this->_frame_height), CV_16SC2, undistmap1r, undistmap2r);
        this->_is_calcTransform_flag = true;
    }

    bool StereoVO::_is_calced_Transform()
    {
        return _is_calcTransform_flag;
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
} // namespace stereo_vo
