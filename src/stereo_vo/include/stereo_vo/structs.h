#ifndef _STEREO_VO_STRUCTS_H
#define _STEREO_VO_STRUCTS_H

#include "./common_includes.h"

namespace stereo_vo
{
    struct Frame;
    struct MapPoint;
    struct Feature;

    struct CameraParameters
    {
        cv::Mat camera_matrix_left;
        cv::Mat camera_matrix_right;
        cv::Mat distortion_coefficients_left;
        cv::Mat distortion_coefficients_right;
        cv::Mat translation_of_camera_right;
        cv::Mat rotation_of_camera_right;
        double baseline;

        cv::Mat Rl, Rr, Pl, Pr, Q;

        CameraParameters(){};
        CameraParameters(
            cv::Mat camera_matrix_left,
            cv::Mat camera_matrix_right,
            cv::Mat distortion_coefficients_left,
            cv::Mat distortion_coefficients_right,
            cv::Mat translation_of_camera_right,
            cv::Mat rotation_of_camera_right)
        {
            this->distortion_coefficients_right = distortion_coefficients_right;
            this->distortion_coefficients_left = distortion_coefficients_left;
            this->camera_matrix_left = camera_matrix_left;
            this->camera_matrix_right = camera_matrix_right;
            this->translation_of_camera_right = translation_of_camera_right;
            this->rotation_of_camera_right = rotation_of_camera_right;
        };
    };

    struct MapPoint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<MapPoint> Ptr;
        unsigned long id_ = 0;
        bool is_outlier_ = false;
        Vec3 pos_ = Vec3::Zero();
        std::mutex data_mutex_;
        int observed_times_ = 0;
        std::list<std::weak_ptr<Feature>> observations_;

        MapPoint() {}

        MapPoint(long id, Vec3 position);

        Vec3 Pos()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        void SetPos(const Vec3 &pos)
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            pos_ = pos;
        };

        void AddObservation(std::shared_ptr<Feature> feature)
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            observations_.push_back(feature);
            observed_times_++;
        }

        void RemoveObservation(std::shared_ptr<Feature> feat);

        std::list<std::weak_ptr<Feature>> GetObs()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return observations_;
        }

        static MapPoint::Ptr CreateNewMappoint();
    };

    struct Feature
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Feature> Ptr;

        std::weak_ptr<Frame> frame_;
        cv::KeyPoint position_;
        DescType descriptor;
        std::weak_ptr<MapPoint> map_point_;

        bool is_outlier_ = false;
        bool is_on_left_image_ = true;

    public:
        Feature() {}

        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
            : frame_(frame), position_(kp) {}
        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp, const DescType &desc)
            : frame_(frame), position_(kp), descriptor(desc) {}
    };

    struct Frame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;
        unsigned long keyframe_id_ = 0;
        bool is_keyframe_ = false;
        double time_stamp_;
        SE3 pose_;
        std::mutex pose_mutex_;
        cv::Mat left_img_, right_img_;

        std::vector<std::shared_ptr<Feature>> features_left_;
        std::vector<std::shared_ptr<Feature>> features_right_;

    public:
        Frame() {}

        Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &left,
              const cv::Mat &right);

        SE3 Pose()
        {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }

        void SetPose(const SE3 &pose)
        {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        void SetKeyFrame();

        static std::shared_ptr<Frame> CreateFrame();
    };

    enum class FrontendStatus
    {
        INITING,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };

} // namespace stereo_vo

#endif // _STEREO_VO_STRUCTS_H