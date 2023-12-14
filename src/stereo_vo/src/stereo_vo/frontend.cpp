#include "../../include/stereo_vo/frontend.h"

namespace stereo_vo
{
    Frontend::Frontend(const int num_features, const int num_features_init)
    {
        this->num_features_init_ = num_features_init;
        this->num_features_ = num_features;
    }

    bool Frontend::AddFrame(stereo_vo::Frame::Ptr frame)
    {
        current_frame_ = frame;
        switch (status_)
        {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            std::cout << "Tracking..." << std::endl;
            break;
        case FrontendStatus::LOST:
            Reset();
            std::cout << "Lost reset." << std::endl;
            break;
        }

        last_frame_ = current_frame_;
        return true;
    }

    bool Frontend::Track()
    {
        if (last_frame_)
        {
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        }

        int num_track_last = TrackLastFrame();
        tracking_inliers_ = EstimateCurrentPose();
        std::cout << "tracking_inliers_: " << tracking_inliers_ << std::endl;

        if (tracking_inliers_ > num_features_tracking_)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else if (tracking_inliers_ > num_features_tracking_bad_)
        {
            status_ = FrontendStatus::TRACKING_BAD;
        }
        else
        {
            status_ = FrontendStatus::LOST;
        }

        InsertKeyframe();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();
        // if (viewer_)
        //     viewer_->AddCurrentFrame(current_frame_);
        return true;
    }

    bool Frontend::InsertKeyframe()
    {
        if (tracking_inliers_ >= num_features_needed_for_keyframe_)
        {
            return false;
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        SetObservationsForKeyFrame();
        DetectFeatures();

        FindFeaturesInRight();
        TriangulateNewPoints();
        // backend_->UpdateMap();

        // if (viewer_)
        //     viewer_->UpdateMap();

        return true;
    }

    void Frontend::SetObservationsForKeyFrame()
    {
        for (auto &feat : current_frame_->features_left_)
        {
            auto mp = feat->map_point_.lock();
            if (mp)
                mp->AddObservation(feat);
        }
    }

    int Frontend::TriangulateNewPoints()
    {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        SE3 current_pose_Twc = current_frame_->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_right_[i] != nullptr)
            {
                // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                        Vec2(current_frame_->features_left_[i]->position_.pt.x,
                             current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                        Vec2(current_frame_->features_right_[i]->position_.pt.x,
                             current_frame_->features_right_[i]->position_.pt.y))};
                Vec3 pworld = Vec3::Zero();

                if (triangulation(poses, points, pworld) && pworld[2] > 0)
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(
                        current_frame_->features_left_[i]);
                    new_map_point->AddObservation(
                        current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        return cnt_triangulated_pts;
    }

    int Frontend::EstimateCurrentPose()
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
            LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<BlockSolverType>(
                std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.addVertex(vertex_pose);

        Mat33 K = camera_left_->K();

        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            auto mp = current_frame_->features_left_[i]->map_point_.lock();
            if (mp)
            {
                features.push_back(current_frame_->features_left_[i]);
                EdgeProjectionPoseOnly *edge =
                    new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(
                    toVec2(current_frame_->features_left_[i]->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        const double chi2_th = 5.991;
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration)
        {
            vertex_pose->setEstimate(current_frame_->Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i];
                if (features[i]->is_outlier_)
                {
                    e->computeError();
                }
                if (e->chi2() > chi2_th)
                {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    cnt_outlier++;
                }
                else
                {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                };

                if (iteration == 2)
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }
        std::cout << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
                  << features.size() - cnt_outlier << std::endl;
        current_frame_->SetPose(vertex_pose->estimate());

        std::cout << "Current Pose = \n"
                  << current_frame_->Pose().matrix() << std::endl;

        for (auto &feat : features)
        {
            if (feat->is_outlier_)
            {
                feat->map_point_.reset();
                feat->is_outlier_ = false;
            }
        }
        return features.size() - cnt_outlier;
    }

    int Frontend::TrackLastFrame()
    {
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp : last_frame_->features_left_)
        {
            if (kp->map_point_.lock())
            {
                auto mp = kp->map_point_.lock();
                auto px =
                    camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_, kps_last,
            kps_current, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }
        cv::Mat imgshow;
        std::vector<cv::KeyPoint> keyPoints;
        cv::KeyPoint::convert(kps_current, keyPoints);
        cv::drawKeypoints(current_frame_->left_img_, keyPoints, imgshow);
        cv::imshow("FLKResult", imgshow);
        cv::waitKey(1);
        return num_good_pts;
    }

    bool Frontend::StereoInit()
    {
        int num_features_left = DetectFeatures();
        int num_coor_features = FindFeaturesInRight();
        if (num_coor_features < num_features_init_)
        {
            return false;
        }

        bool build_map_success = BuildInitMap();
        if (build_map_success)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
            // if (viewer_)
            // {
            //     viewer_->AddCurrentFrame(current_frame_);
            //     viewer_->UpdateMap();
            // }
            return true;
        }
        return false;
    }

    int Frontend::DetectFeatures()
    {
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame_->features_left_)
        {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
        }
        std::vector<cv::KeyPoint> keypoints;
        std::vector<DescType> descriptor;
        cv::FAST(current_frame_->left_img_, keypoints, 40);
        ComputeORB(current_frame_->left_img_, keypoints, descriptor);
        int cnt_detected = 0;
        for (auto &kp : keypoints)
        {
            current_frame_->features_left_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }
        std::cout << "Detect " << cnt_detected << " new features" << std::endl;
        cv::Mat imgshow;
        cv::drawKeypoints(current_frame_->left_img_, keypoints, imgshow);
        cv::imshow("ORBResult", imgshow);
        cv::waitKey(1);
        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight()
    {
        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp : current_frame_->features_left_)
        {
            kps_left.push_back(kp->position_.pt);
            auto mp = kp->map_point_.lock();
            if (mp)
            {
                auto px =
                    camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_right.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_, kps_left,
            kps_right, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                current_frame_->features_right_.push_back(nullptr);
            }
        }

        std::cout << "Find " << num_good_pts << " in the right image." << std::endl;
        cv::Mat imgshow;
        std::vector<cv::KeyPoint> keyPoints;
        cv::KeyPoint::convert(kps_right, keyPoints);
        cv::drawKeypoints(current_frame_->right_img_, keyPoints, imgshow);
        cv::imshow("FLKResult_right", imgshow);
        cv::waitKey(1);

        return num_good_pts;
    }

    bool Frontend::BuildInitMap()
    {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_right_[i] == nullptr)
                continue;
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                cnt_init_landmarks++;
                map_->InsertMapPoint(new_map_point);
            }
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        // backend_->UpdateMap();

        return true;
    }

    bool Frontend::Reset()
    {
        status_ = FrontendStatus::INITING;
        return true;
    }

    void Frontend::ComputeORB(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, std::vector<DescType> &descriptors)
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
                    cv::Point2f p(ORB_pattern[idx_pq * 4], ORB_pattern[idx_pq * 4 + 1]);
                    cv::Point2f q(ORB_pattern[idx_pq * 4 + 2], ORB_pattern[idx_pq * 4 + 3]);

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

        std::cout << "bad/total: " << bad_points << "/" << keypoints.size() << std::endl;
    }

    void Frontend::BfMatch(const std::vector<DescType> &desc1, const std::vector<DescType> &desc2, std::vector<cv::DMatch> &matches)
    {
        const int d_max = 40;

        for (size_t i1 = 0; i1 < desc1.size(); ++i1)
        {
            if (desc1[i1].empty())
                continue;
            cv::DMatch m{i1, 0, 256};
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
}