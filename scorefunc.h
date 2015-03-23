#ifndef SCOREFUNC_H
#define SCOREFUNC_H
#include "commonFunc.h"
class scoreFunc
{
public:
    scoreFunc();

    // Ransac for outlier rejection
    PointCloudT::Ptr motFeatRansac(const PointCloudT::Ptr &motFeats);

    // estimate the motion of object with rotation and translation
    f32 motEstDist(const Eigen::Matrix4Xf &transMat, const PointT refPt,
                   const PointT motPt);

    // transform point cloud
    void transformPC(const Eigen::Matrix4f transMat,
                     const PointCloudT::Ptr & pcInput,
                     PointCloudT::Ptr & pcOutput);

    // color distance
    void colorDist(const PointT refPt, const PointT motPt);

    // descriptor distance
    void descriptorDist();
};

#endif // SCOREFUNC_H
