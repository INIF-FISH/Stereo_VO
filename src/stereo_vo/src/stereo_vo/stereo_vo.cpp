#include "../../include/stereo_vo/stereo_vo.h"

namespace stereo_vo
{
    StereoVO::StereoVO()
    {
    }

    StereoVO::~StereoVO()
    {
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

    void StereoVO::addFrame(cv::Mat &img_l, cv::Mat &img_r)
    {
        assert(img_l.data != nullptr && img_r.data != nullptr);
        
    }
} // namespace stereo_vo
