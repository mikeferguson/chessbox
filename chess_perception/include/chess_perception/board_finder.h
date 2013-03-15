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
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <image_transport/image_transport.h>

/** \class BoardFinder
 *  \brief Finds potential points of the chess board.
 */
class BoardFinder
{
  public:
    BoardFinder();
    virtual ~BoardFinder() {};

    /** \brief Find the potential points in the image. */
    bool findCorners(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                     boost::shared_ptr< std::vector<pcl::PointXYZ> > points);

  private:
    /** \brief Helper function to decide whether to accept a point */
    bool accept(cv::Point& p, std::vector<cv::Point>& points);

    /** \brief Helper function to decide whether to accept a 3d point */
    bool accept_3d(pcl::PointXYZ& p, std::vector<pcl::PointXYZ>& points);

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

    /* configuration */
    bool debug_;

    image_transport::Publisher pub_;
    cv_bridge::CvImagePtr bridge_;
};

#endif
