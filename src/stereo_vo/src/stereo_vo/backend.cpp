#include "../../include/stereo_vo/backend.h"

namespace stereo_vo
{
    Backend::Backend()
    {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

    void Backend::UpdateMap()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
    }

    void Backend::Stop()
    {
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop()
    {
        while (backend_running_.load())
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);

            Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            Optimize(active_kfs, active_landmarks);
        }
    }

    void Backend::Optimize(Map::KeyframesType &keyframes,
                           Map::LandmarksType &landmarks)
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
            LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<BlockSolverType>(
                std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes)
        {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);

            max_kf_id = std::max(max_kf_id,kf->keyframe_id_);
            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        int index = 1;
        double chi2_th = 5.991;
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        for (auto &landmark : landmarks)
        {
            if (landmark.second->is_outlier_)
                continue;
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetObs();
            for (auto &obs : observations)
            {
                if (obs.lock() == nullptr)
                    continue;
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
                    continue;

                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_image_)
                {
                    edge = new EdgeProjection(K, left_ext);
                }
                else
                {
                    edge = new EdgeProjection(K, right_ext);
                }

                if (vertices_landmarks.find(landmark_id) ==
                    vertices_landmarks.end())
                {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                if (vertices.find(frame->keyframe_id_) !=
                        vertices.end() &&
                    vertices_landmarks.find(landmark_id) !=
                        vertices_landmarks.end())
                {
                    edge->setId(index);
                    edge->setVertex(0, vertices.at(frame->keyframe_id_));
                    edge->setVertex(1, vertices_landmarks.at(landmark_id));
                    edge->setMeasurement(toVec2(feat->position_.pt));
                    edge->setInformation(Mat22::Identity());
                    auto rk = new g2o::RobustKernelHuber();
                    rk->setDelta(chi2_th);
                    edge->setRobustKernel(rk);
                    edges_and_features.insert({edge, feat});
                    optimizer.addEdge(edge);
                    index++;
                }
                else
                    delete edge;
            }
        }

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5)
        {
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;
            cnt_inlier = 0;

            for (auto &ef : edges_and_features)
            {
                if (ef.first->chi2() > chi2_th)
                {
                    cnt_outlier++;
                }
                else
                {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5)
            {
                break;
            }
            else
            {
                chi2_th *= 2;
                iteration++;
            }
        }

        for (auto &ef : edges_and_features)
        {
            if (ef.first->chi2() > chi2_th)
            {
                ef.second->is_outlier_ = true;
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
                ef.second->map_point_.reset();
            }
            else
            {
                ef.second->is_outlier_ = false;
            }
        }

        std::cout << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
                  << cnt_inlier << std::endl;

        for (auto &v : vertices)
        {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks)
        {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }
} // namspace stereo_vo