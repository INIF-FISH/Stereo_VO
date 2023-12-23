#ifndef _STEREO_VO_H
#define _STEREO_VO_H

#include "./common_includes.h"
#include "./structs.h"
#include "./frontend.h"

namespace stereo_vo
{
    class StereoVO
    {
    private:
        CameraParameters params;
        int _frame_width = 0, _frame_height = 0;
        cv::Mat undistmap1l, undistmap2l, undistmap1r, undistmap2r;

        bool _is_calcTransform_flag = false;
        bool _params_init_flag = false;

        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;

    public:
        bool _ifShowErr = false;

    public:
        StereoVO();
        ~StereoVO();

        void init();
        void stop();
        bool read_param(const std::string &path, const int frame_width, const int frame_height);
        bool _is_params_inited();
        void calcTransform();
        bool _is_calced_Transform();

    public:
        bool addFrame(cv::Mat &img_l, cv::Mat &img_r);
        bool getPose(Eigen::Matrix3d &rotation_matrix, Eigen::Vector3d &translation_vector);
    };
} // namespace stereo_vo

#endif // _STEREO_VO_H