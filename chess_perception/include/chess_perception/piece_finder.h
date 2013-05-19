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

#ifndef PIECE_FINDER_H_
#define PIECE_FINDER_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

/** \class PieceFinder
 *  \brief Finds the pieces.
 */
class PieceFinder
{
  public:
    PieceFinder();
    virtual ~PieceFinder() {};

    /** \brief Finds pieces on a table.
     *
     *  \returns number of pieces found.
     */
    int findPieces(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                   tf::Transform& board_transform,
                   std::vector<pcl::PointXYZ>& pieces,
                   std::vector<double>& weights);
    
  private:
    bool debug_;
    int threshold_;

    pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> extract_data_;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_;

    ros::Publisher pieces_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZRGB> hull_untransformed_;
};

#endif
