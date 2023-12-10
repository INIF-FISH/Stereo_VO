#include "../../include/stereo_vo/structs.h"

namespace stereo_vo
{
    Frame::Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &left, const cv::Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

    Frame::Ptr Frame::CreateFrame()
    {
        static long factory_id = 0;
        Frame::Ptr new_frame(new Frame);
        new_frame->id_ = factory_id++;
        return new_frame;
    }

    void Frame::SetKeyFrame()
    {
        static long keyframe_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }

    MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}

    MapPoint::Ptr MapPoint::CreateNewMappoint()
    {
        static long factory_id = 0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++;
        return new_mappoint;
    }

    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        for (auto iter = observations_.begin(); iter != observations_.end();
             iter++)
        {
            if (iter->lock() == feat)
            {
                observations_.erase(iter);
                feat->map_point_.reset();
                observed_times_--;
                break;
            }
        }
    }
} // namespace stereo_vo