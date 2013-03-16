/**

Copyright (c) 2011-2013 Michael E. Ferguson.  All right reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 

**/

#ifndef TABLE_FINDER_H_
#define TABLE_FINDER_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

/** \class TableFinder
 *  \brief Given a point cloud input, this class finds the table.
 */
class TableFinder
{
  public:
    TableFinder();
    virtual ~TableFinder() {};

    /** \brief Finds a table.
     *  Algorithm: downsamples the cloud using a voxel filter. Computes
     *  normals on the downsampled cloud, runs SAC segmentation using normals.
     *  Projects inliers onto the plane and finds a convex hull.
     *
     *  \returns true if table is found.
     */
    bool findTable(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull);

  private:
    bool debug_;

    /** \brief Size of downsampled voxel grid. */
    double leaf_size_;

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normals_;
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentation_;
    pcl::ProjectInliers<pcl::PointXYZRGB> project_;
    pcl::ConvexHull<pcl::PointXYZRGB> convexhull_;

    ros::Publisher table_cloud_pub_;
    ros::Publisher hull_cloud_pub_;
};

#endif
