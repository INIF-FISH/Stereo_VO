#include "../../include/stereo_vo/frontend.h"

namespace stereo_vo
{
    Frontend::Frontend(const int num_features, const int num_features_init)
    {
        this->num_features_init_ = num_features_init;
        this->num_features_ = num_features;
        this->_gftt = cv::GFTTDetector::create(num_features, 0.01, 1.0, 3, false);
    }

    bool Frontend::AddFrame(stereo_vo::Frame::Ptr frame)
    {
        this->camera_left_->UndistortImage(frame->left_img_, frame->left_img_);
        this->camera_right_->UndistortImage(frame->right_img_, frame->right_img_);
        current_frame_ = frame;
        cv::imshow("tl", frame->left_img_);
        cv::imshow("tr", frame->right_img_);
        cv::waitKey(1);
        switch (status_)
        {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
        }

        last_frame_ = current_frame_;
        return true;
    }

    void Frontend::SetMaxDepth(const double &value)
    {
        this->max_depth = value;
    }

    bool Frontend::GetPose(Eigen::Matrix4d &pose)
    {
        if (current_frame_)
        {
            pose = current_frame_->Pose().matrix();
            return true;
        }
        return false;
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
            std::cout << "========================================================" << std::endl;
            std::cout << "==========================LOST==========================" << std::endl;
            std::cout << "========================================================" << std::endl;
        }
        if (status_ != FrontendStatus::LOST)
            InsertKeyframe();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();
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
        backend_->UpdateMap();

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
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                        Vec2(current_frame_->features_left_[i]->position_.pt.x,
                             current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                        Vec2(current_frame_->features_right_[i]->position_.pt.x,
                             current_frame_->features_right_[i]->position_.pt.y))};
                Vec3 pcamera = Vec3::Zero();

                if (triangulation(poses, points, pcamera) && pcamera[2] > 0)
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    new_map_point->SetPos(current_pose_Twc * pcamera);
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
        solver->setUserLambdaInit(1e-6);
        solver->setMaxTrialsAfterFailure(10);
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
        int max_iteration = 8;
        for (int iteration = 0; iteration < max_iteration; ++iteration)
        {
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
                }
                if (max_iteration - iteration == 2)
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
            kps_current, status, error, cv::Size(31, 31), 7,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        cv::Mat inliersMask;
        cv::Mat H = cv::findHomography(kps_last, kps_current, inliersMask, cv::RANSAC, 3.3);

        int num_good_pts = 0;

        cv::Mat show = current_frame_->left_img_.clone();
        cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);

        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i] && inliersMask.at<char>(i))
            {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
                cv::line(show, kps_current[i], kps_last[i], cv::Scalar(0, 0, 255));
                cv::circle(show, kps_current[i], 2, cv::Scalar(0, 255, 0));
            }
        }

        cv::imshow("Track", show);
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
        this->_gftt->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0;
        for (auto &kp : keypoints)
        {
            current_frame_->features_left_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }
        cv::Mat show = current_frame_->left_img_.clone();
        cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
        cv::drawKeypoints(show, keypoints, show, cv::Scalar(255, 0, 0));
        cv::imshow("Detect", show);
        cv::waitKey(1);
        std::cout << "Detect " << cnt_detected << " new features" << std::endl;
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
            kps_right, status, error, cv::Size(31, 31), 7,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        cv::Mat show = cv::Mat(current_frame_->left_img_.rows, current_frame_->left_img_.cols + current_frame_->right_img_.cols, current_frame_->left_img_.type(), cv::Scalar::all(0));

        current_frame_->left_img_.copyTo(show(cv::Rect(0, 0, current_frame_->left_img_.cols, current_frame_->left_img_.rows)));
        current_frame_->right_img_.copyTo(show(cv::Rect(current_frame_->left_img_.cols, 0, current_frame_->left_img_.cols, current_frame_->left_img_.rows)));

        cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                if (abs(kps_left[i].y - kps_right[i].y) / (kps_left[i].x - kps_right[i].x) > 0.3 || kps_left[i].x - kps_right[i].x < 0)
                {
                    status[i] = false;
                    current_frame_->features_right_.push_back(nullptr);
                    continue;
                }
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
                cv::line(show, kps_left[i], kps_right[i] + cv::Point2f(current_frame_->left_img_.cols, 0), cv::Scalar(0, 0, 255));
                cv::circle(show, kps_left[i], 2, cv::Scalar(0, 255, 0));
                cv::circle(show, kps_right[i] + cv::Point2f(current_frame_->left_img_.cols, 0), 2, cv::Scalar(255, 0, 0));
            }
            else
            {
                current_frame_->features_right_.push_back(nullptr);
            }
        }

        cv::imshow("Detect Right", show);
        cv::waitKey(1);

        std::cout << "Find " << num_good_pts << " in the right image." << std::endl;

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

            if (triangulation(poses, points, pworld) && pworld[2] > 0 && pworld[2] < this->max_depth)
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
        backend_->UpdateMap();

        return true;
    }

    bool Frontend::Reset()
    {
        return true;
    }
} // namespace stereo_vo