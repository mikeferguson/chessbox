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

#ifndef POINT_FINDER_H_
#define POINT_FINDER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h> // TODO: find appropriate include file

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chess_perception/conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <image_transport/image_transport.h>

#include <pcl/registration/transformation_estimation_svd.h>

/** \class BoardFinder
 *  \brief Finds potential points of the chess board.
 */
class BoardFinder
{
  public:
    BoardFinder();
    virtual ~BoardFinder() {};

    /** \brief Find the potential points in the image. */
    bool findBoard(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                   tf::Transform& board);

    /** \brief Set the size of a square on our chess board. */
    void setSquareSize(double size) { square_size_ = size; }

  private:
    /** \brief Helper function to decide whether to accept a point */
    bool accept(cv::Point& p, std::vector<cv::Point>& points);

    /** \brief Helper function to decide whether to accept a 3d point */
    bool accept_3d(pcl::PointXYZRGB& p, pcl::PointCloud<pcl::PointXYZRGB>& cloud);

    /** \brief Helper function to find intersection of two lines */ 
    cv::Point findIntersection( cv::Vec4i a, cv::Vec4i b );

    /* parameters for segmentation */
    int channel_;

    /* parameters for hough line detection */
    int h_rho_;
    int h_threshold_;
    int h_min_length_;

    /* parameters for point detection */
    int point_threshold_;

    /* board parameters -- the size of a square */
    double square_size_;

    /* configuration */
    bool debug_;

    ros::Publisher cloud_pub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr bridge_;

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZ> reg_;
};

#endif
