#ifndef LSL_POINT_CLOUD_UTILS
#define LSL_POINT_CLOUD_UTILS

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace lslgeneric
{
/* \brief Routines to read/write point clouds from VRML files */

template< typename PointT>
pcl::PointCloud<PointT> transformPointCloud(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T,
        const pcl::PointCloud<PointT> &pc);
template< typename PointT>
void transformPointCloudInPlace(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T, pcl::PointCloud<PointT> &pc);

template< typename PointT>
double geomDist(PointT p1, PointT p2);

};
#include<ndt_map/impl/pointcloud_utils.hpp>

#endif

