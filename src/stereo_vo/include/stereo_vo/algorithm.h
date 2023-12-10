#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include "./common_includes.h"

namespace stereo_vo
{
    inline bool triangulation(const std::vector<SE3> &poses,
                              const std::vector<Vec3> points, Vec3 &pt_world)
    {
        MatXX A(2 * poses.size(), 4);
        VecX b(2 * poses.size());
        b.setZero();
        for (size_t i = 0; i < poses.size(); ++i)
        {
            Mat34 m = poses[i].matrix3x4();
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
        }
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

        if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
        {
            return true;
        }
        return false;
    }

    inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }
} // namespace stereo_vo

#endif // _ALGORITHM_H