#include "scorefunc.h"

scoreFunc::scoreFunc()
{
}

/////////////////////////////////////////////////////////////////////////////////////
/// Func to estimate the point distance after motion estimation
/////////////////////////////////////////////////////////////////////////////////////
/// transMat: estimated motion matrix
/// motPt: feature position after motion
/// refPt: reference feature position (first frame)
///
f32 scoreFunc::motEstDist(const Eigen::Matrix4Xf &transMat, const PointT refPt,
                          const PointT motPt)
{
    Eigen::Vector4f _motPt(motPt.x, motPt.y, motPt.z, 1.0);
    // transform the point cloud
    Eigen::Vector4f _motPtTrans = transMat*_motPt;
    // calculate the l2 norm distance
    return commonFunc::l2norm(_motPtTrans(0) - refPt.x, _motPtTrans(1) - refPt.y,
                              _motPtTrans(2) - refPt.z);
}


/////////////////////////////////////////////////////////////////////////////////////
/// Func to rigidly transform the point cloud using a transformation matrix
/////////////////////////////////////////////////////////////////////////////////////
void scoreFunc::transformPC(const Eigen::Matrix4f transMat,
                            const PointCloudT::Ptr &pcInput,
                            PointCloudT::Ptr &pcOutput)
{// transform point cloud
    pcl::transformPointCloud(*pcInput, *pcOutput, transMat);
}


