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

    void StereoVO::ComputeORB(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, std::vector<DescType> &descriptors)
    {
        const int half_patch_size = 8;
        const int half_boundary = 16;
        int bad_points = 0;
        for (auto &kp : keypoints)
        {
            if (kp.pt.x < half_boundary || kp.pt.y < half_boundary ||
                kp.pt.x >= img.cols - half_boundary || kp.pt.y >= img.rows - half_boundary)
            {
                // outside
                bad_points++;
                descriptors.push_back({});
                continue;
            }

            float m01 = 0, m10 = 0;
            for (int dx = -half_patch_size; dx < half_patch_size; ++dx)
            {
                for (int dy = -half_patch_size; dy < half_patch_size; ++dy)
                {
                    uchar pixel = img.at<uchar>(kp.pt.y + dy, kp.pt.x + dx);
                    m10 += dx * pixel;
                    m01 += dy * pixel;
                }
            }

            // angle should be arc tan(m01/m10);
            float m_sqrt = sqrt(m01 * m01 + m10 * m10) + 1e-18; // avoid divide by zero
            float sin_theta = m01 / m_sqrt;
            float cos_theta = m10 / m_sqrt;

            // compute the angle of this point
            DescType desc(8, 0);
            for (int i = 0; i < 8; i++)
            {
                uint32_t d = 0;
                for (int k = 0; k < 32; k++)
                {
                    int idx_pq = i * 32 + k;
                    cv::Point2f p(this->ORB_pattern[idx_pq * 4], this->ORB_pattern[idx_pq * 4 + 1]);
                    cv::Point2f q(this->ORB_pattern[idx_pq * 4 + 2], this->ORB_pattern[idx_pq * 4 + 3]);

                    // rotate with theta
                    cv::Point2f pp = cv::Point2f(cos_theta * p.x - sin_theta * p.y, sin_theta * p.x + cos_theta * p.y) + kp.pt;
                    cv::Point2f qq = cv::Point2f(cos_theta * q.x - sin_theta * q.y, sin_theta * q.x + cos_theta * q.y) + kp.pt;
                    if (img.at<uchar>(pp.y, pp.x) < img.at<uchar>(qq.y, qq.x))
                    {
                        d |= 1 << k;
                    }
                }
                desc[i] = d;
            }
            descriptors.push_back(desc);
        }
    }

    void StereoVO::BfMatch(const std::vector<DescType> &desc1, const std::vector<DescType> &desc2, std::vector<cv::DMatch> &matches)
    {
        const int d_max = 40;

        for (size_t i1 = 0; i1 < desc1.size(); ++i1)
        {
            if (desc1[i1].empty())
                continue;
            cv::DMatch m{int(i1), 0, 256};
            for (size_t i2 = 0; i2 < desc2.size(); ++i2)
            {
                if (desc2[i2].empty())
                    continue;
                int distance = 0;
                for (int k = 0; k < 8; k++)
                {
                    distance += _mm_popcnt_u32(desc1[i1][k] ^ desc2[i2][k]);
                }
                if (distance < d_max && distance < m.distance)
                {
                    m.distance = distance;
                    m.trainIdx = i2;
                }
            }
            if (m.distance < d_max)
            {
                matches.push_back(m);
            }
        }
    }

    void StereoVO::cleanUP()
    {
        this->keypoints_l.clear();
        this->keypoints_r.clear();
        this->descriptor_l.clear();
        this->descriptor_r.clear();
        this->matches.clear();
    }

    void StereoVO::addFrame(cv::Mat &img_l, cv::Mat &img_r)
    {
        assert(img_l.data != nullptr && img_r.data != nullptr);
        this->cleanUP();
        cv::FAST(img_l, this->keypoints_l, 40);
        cv::FAST(img_r, this->keypoints_r, 40);
        this->ComputeORB(img_l, this->keypoints_l, this->descriptor_l);
        this->ComputeORB(img_r, this->keypoints_r, this->descriptor_r);
        this->BfMatch(this->descriptor_l, this->descriptor_r, this->matches);

        if (this->keypoints_l.size() > 0 && this->keypoints_r.size() > 0)
        {
            cv::Mat image_show;
            cv::cvtColor(img_l, img_l, cv::COLOR_GRAY2BGR);
            cv::cvtColor(img_r, img_r, cv::COLOR_GRAY2BGR);
            cv::drawMatches(img_l, this->keypoints_l, img_r, this->keypoints_r, this->matches, image_show);
            cv::imshow("matches", image_show);
            cv::waitKey(1);
        }
    }
} // namespace stereo_vo
