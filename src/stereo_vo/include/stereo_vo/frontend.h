#ifndef _FRONTEND_H
#define _FRONTEND_H

#include "./common_includes.h"
#include "./structs.h"
#include "./map.h"
#include "./camera_vo.h"
#include "./algorithm.h"
#include "./g2o_types.h"
#include "./backend.h"

namespace stereo_vo
{
    class Frontend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend(const int num_features, const int num_features_init);

        bool AddFrame(Frame::Ptr frame);

        void SetMap(Map::Ptr map) { map_ = map; }

        void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

        bool GetPose(Eigen::Matrix4d &pose);

        FrontendStatus GetStatus() const { return status_; }

        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            camera_left_ = left;
            camera_right_ = right;
        }

    private:
        bool Track();
        bool Reset();
        int TrackLastFrame();
        int EstimateCurrentPose();
        bool InsertKeyframe();
        bool StereoInit();
        int DetectFeatures();
        int FindFeaturesInRight();
        bool BuildInitMap();
        int TriangulateNewPoints();
        void SetObservationsForKeyFrame();

        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;
        Frame::Ptr last_frame_ = nullptr;
        Camera::Ptr camera_left_ = nullptr;
        Camera::Ptr camera_right_ = nullptr;

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;

        cv::Ptr<cv::GFTTDetector> _gftt;

        SE3 relative_motion_;

        int tracking_inliers_ = 0;

        int num_features_ = 200;
        int num_features_init_ = 100;
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframe_ = 80;
    };
} // namespace stereo_vo

#endif // _FRONTEND_H