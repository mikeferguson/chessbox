/**

Copyright (c) 2011-2021 Michael E. Ferguson.  All right reserved.

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

#include <chess_perception/board_finder.h>

#include <ros/ros.h>
#include <math.h>

/** @brief Helper function to convert Eigen transformation to tf -- thanks to Garret Gallagher */
tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
  tf::Matrix3x3 mat;
  mat.setValue(trans(0,0),trans(0,1),trans(0,2),
               trans(1,0),trans(1,1),trans(1,2),
               trans(2,0),trans(2,1),trans(2,2));
  tf::Transform ret;
  ret.setOrigin(tf::Vector3(trans(0,3),trans(1,3),trans(2,3)));
  ret.setBasis(mat);
  return ret;
}

/** @brief Sort points based only on x-component */
bool orderPointsX(const cv::Point& a, const cv::Point& b)
{
  return a.x < b.x;
}

/** @brief Order horizontal lines (bottom of screen first) */
bool orderLinesH(const cv::Vec4i& a, const cv::Vec4i& b)
{
  double ma = (a[3] - a[1]) / (double)(a[2] - a[0]);
  double mb = (b[3] - b[1]) / (double)(b[2] - b[0]);
  double ba = a[1] - ma*a[0];
  double bb = b[1] - mb*b[0];
  return ba > bb;
}

/** Used to determine distance between two points */
double point_distance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2)
{
  return sqrt(pow(p1.z - p2.z, 2) + pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

BoardFinder::BoardFinder()
{
  ros::NodeHandle nh ("~");

  // Load parameters
  if (!nh.getParam ("board_color", channel_))
  {
    channel_ = 0; // blue
  }
  if (!nh.getParam ("point_threshold", point_threshold_))
  {
    point_threshold_ = 42;
  }

  // Load parameters for hough transform
  if (!nh.getParam ("h_rho", h_rho_))
  {
    h_rho_ = 1;
  }
  ROS_INFO ("Hough Rho: %d", h_rho_);

  if (!nh.getParam ("h_threshold", h_threshold_))
  {
    h_threshold_ = 50;
  }
  ROS_INFO ("Hough Threshold: %d", h_threshold_);

  if (!nh.getParam ("h_min_length", h_min_length_))
  {
    h_min_length_ = 100;
  }
  ROS_INFO ("Hough Min Length: %d", h_min_length_);

  debug_ = true;

  // Debug image output
  if (debug_)
  {
    image_transport::ImageTransport it(nh);
    image_pub_ = it.advertise("board_finder_image", 1);
    cloud_pub_ = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("board_finder_cloud", 1);
  }
}

bool BoardFinder::findBoard(
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
  tf::Transform& board)
{
  // Get an OpenCV image from the cloud
  sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
  pcl_broke_again::toROSMsg(*cloud, *image_msg);
  try
  {
    bridge_ = cv_bridge::toCvCopy(image_msg, "rgb8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Conversion failed");
  }

  // Segment based on a channel (blue board squares)
  cv::Mat dst, cdst;
  cv::Mat src(bridge_->image.rows, bridge_->image.cols, CV_8UC1);
  for (int i = 0; i < bridge_->image.rows; ++i)
  {
    const char* Di = bridge_->image.ptr<char>(i);
    char* Ii = src.ptr<char>(i);
    for (int j = 0; j < bridge_->image.cols; ++j)
    {
      Ii[j + channel_] = Di[j * 3 + channel_];
    }
  }

  // Threshold, erode/dilate to clean up image
  cv::threshold(src, src, 120, 255, cv::THRESH_BINARY);
  cv::erode(src, src, cv::Mat());
  cv::dilate(src, src, cv::Mat());
  // Edge detection, dilation before hough transform
  cv::Canny(src, dst, 30, 200, 3);
  cv::dilate(dst, dst, cv::Mat());

  // Do a hough transformation to find lines
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(dst, lines, h_rho_, CV_PI/180, h_threshold_, h_min_length_, 10);
  ROS_DEBUG("Found %d lines", (int) lines.size());

  // Split into vertical/horizontal lines
  std::vector<cv::Vec4i> h_lines, v_lines;
  for (size_t i = 0; i < lines.size(); ++i)
  {
    cv::Vec4i a = lines[i];
    int dx = a[2] - a[0];
    int dy = a[3] - a[1];
    if (abs(dx) > abs(dy))
    {
      bool push = true;
      double ma = (a[3] - a[1]) / (double)(a[2] - a[0]);
      double ba = a[1] - ma*a[0];
      for( size_t j = 0; j < h_lines.size(); j++)
      {
        cv::Vec4i b = h_lines[j];
        double mb = (b[3] - b[1]) / (double)(b[2] - b[0]);
        double bb = b[1] - mb * b[0];
        if ((fabs(ba - bb) < 15.0f) && (fabs(ma - mb) < 0.5))
        {
          push = false;
          if (abs(dx) > abs(b[2] - b[0]))
          {
            h_lines[j] = a;
          }
          break;
        }
      }
      if (push)
      {
        h_lines.push_back(a);
      }
    }
    else
    {
      bool push = true;
      double ma = (a[2] - a[0]) / (double)(a[3] - a[1]);  // x = my+b
      double ba = a[0] - ma*a[1];
      for (size_t j = 0; j < v_lines.size(); ++j)
      {
        cv::Vec4i b = v_lines[j];
        double mb = (b[2] - b[0]) / (double)(b[3] - b[1]);
        double bb = b[0] - mb*b[1];
        if ((fabs(ba-bb) < 15.0f) && (fabs(ma - mb) < 0.5))
        {
          push = false;
          if (abs(dy) > abs(b[3] - b[1]))
          {
            v_lines[j] = a;
          }
          break;
        }
      }
      if (push)
      {
        v_lines.push_back(a);
      }
    }
  }

  // Sort horizontal lines
  std::sort(h_lines.begin(), h_lines.end(), orderLinesH);

  // Output lines to screen
  if (debug_)
  {
    // Convert back to color
    cv::cvtColor(src, cdst, CV_GRAY2BGR);

    // Then blue/green for horizontal/vertical
    ROS_DEBUG("horizontal lines: %d", (int) h_lines.size());
    for (size_t i = 0; i < h_lines.size(); ++i)
    {
      cv::Vec4i l = h_lines[i];
      if (i%2 == 0)
      {
        cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 2, cv::LineTypes::LINE_AA);
      }
      else
      {
        cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,255), 2, cv::LineTypes::LINE_AA);
      }
    }
    ROS_DEBUG("vertical lines: %d", (int) v_lines.size());
    for (size_t i = 0; i < v_lines.size(); ++i)
    {
      cv::Vec4i l = v_lines[i];
      cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 2, cv::LineTypes::LINE_AA);
    }
  }

  // Get all (2d) intersections, which are proper corners
  std::vector<std::vector<cv::Point>> corner_points_2d;
  for (size_t i = 0; i < h_lines.size(); ++i)
  {
    cv::Vec4i hl = h_lines[i];
    std::vector<cv::Point> temp_lines;
    for (size_t j = 0; j < v_lines.size(); ++j)
    {
      // For each vertical and horizontal line pair, find the intersection
      cv::Vec4i vl = v_lines[j];
      cv::Point p = findIntersection(hl, vl);
      // Test that intersection exists, and is not close to a point already picked
      if (p.x > 0 && p.y > 0)
      {
        /* Test if this really is a checkerboard intersection by checking
         * the color of points on four sides.
         */
        unsigned dark_pixels[4] = {0, 0, 0, 0};
        unsigned light_pixels[4] = {0, 0, 0, 0};
        for (int x = 3; x < 12; ++x)
        {
          for (int y = 3; y < 12; ++y)
          {
            if (src.at<unsigned char>(p.y + y, p.x + x) < 120)
            {
              ++dark_pixels[0];
            }
            else
            {
              ++light_pixels[0];
            }

            if (src.at<unsigned char>(p.y + y, p.x - x) < 120)
            {
              ++dark_pixels[1];
            }
            else
            {
              ++light_pixels[1];
            }

            if (src.at<unsigned char>(p.y - y, p.x - x) < 120)
            {
              ++dark_pixels[2];
            }
            else
            {
              ++light_pixels[2];
            }

            if (src.at<unsigned char>(p.y - y, p.x + x) < 120)
            {
              ++dark_pixels[3];
            }
            else
            {
              ++light_pixels[3];
            }
          }
        }

        if (dark_pixels[0] > light_pixels[0])
        {
          if (dark_pixels[1] > light_pixels[1] ||
              dark_pixels[2] < light_pixels[2] ||
              dark_pixels[3] > light_pixels[3])
          {
            if (debug_)
            {
              cv::circle(cdst, p, 5, cv::Scalar(255,255,0), -1);
            }
            continue;
          }
        }
        else
        {
          if (dark_pixels[1] < light_pixels[1] ||
              dark_pixels[2] > light_pixels[2] ||
              dark_pixels[3] < light_pixels[3])
          {
            if (debug_)
            {
              cv::circle(cdst, p, 5, cv::Scalar(255,255,0), -1);
            }
            continue;
          }
        }
        temp_lines.push_back(p);
        if (debug_)
        {
          // Draw circles
          cv::circle(cdst, p, 5, cv::Scalar(255,0,0), -1);
        }
      }
    }
    if (temp_lines.size() >= 5)
    {
      std::stable_sort(temp_lines.begin(), temp_lines.end(), orderPointsX);
      corner_points_2d.push_back(temp_lines);
    }
  }

  if (debug_)
  {
    bridge_->image = cdst;
    image_pub_.publish(bridge_->toImageMsg());
  }

  // We need some data from each row
  if (corner_points_2d.size() < 7)
  {
    ROS_WARN("Board Finder: Missing an entire row");
    return false;
  }

  // Project to 3d, try to handle missing points in center of row
  int corner_indices[7][7];
  for (size_t i = 0; i < 7; ++i)
  {
    for (size_t j = 0; j < 7; ++j)
    {
      corner_indices[i][j] = -1;
    }
  }
  pcl::PointCloud<pcl::PointXYZRGB> points;
  for (size_t i = 0; i < corner_points_2d.size(); ++i)
  {
    int j_offset = 0;
    for (size_t j = 0; j < corner_points_2d[i].size(); ++j)
    {
      cv::Point p2d = corner_points_2d[i][j];
      pcl::PointXYZRGB p3d = (*cloud)(p2d.x, p2d.y);
      if (!isnan(p3d.x) && !isnan(p3d.y) && !isnan(p3d.z))
      {
        if (accept_3d(p3d, points))
        {
          points.push_back(p3d);
          // Insert index
          if (j > 0)
          {
            if (point_distance(p3d, points[points.size() - 2]) > (square_size_ * 1.3))
            {
              j_offset += (int) (point_distance(p3d, points[points.size() - 2]) / square_size_) - 0.5;
            }
            if (j + j_offset < 7)
            {
              corner_indices[i][j + j_offset] = points.size() - 1;
            }
          }
          else if (j + j_offset < 7)
          {
            corner_indices[i][j + j_offset] = points.size() - 1;
          }
        }
        else
        {
          --j_offset;
        }
      }
      // TODO: add some means of searching for a point near here that is not a nan.
    }
  }

  // TODO: handle missing points at the beginning of a row

  ROS_DEBUG_STREAM("Board Finder: Found " << points.size() << " 3d points");

  if (debug_)
  {
    points.header = cloud->header;
    cloud_pub_.publish(points);
  }

  if (static_cast<int>(points.size()) < point_threshold_)
  {
    return false;
  }

  // Estimate board/piece pose
  pcl::PointCloud<pcl::PointXYZ> ideal;
  for (int h = 1; h < 8; ++h)
  {
    for (int v = 1; v < 8; ++v)
    {
      if (corner_indices[h - 1][v - 1] >= 0)
      {
        ideal.push_back(pcl::PointXYZ(square_size_ * v, square_size_ * h, 0));
      }
    }
  }

  // Occasionally, we hit this, mainly due to motion
  if (points.size() != ideal.size())
  {
    ROS_WARN_STREAM("Board Finder: Failed size check: " << points.size() << " vs " << ideal.size());
    return false;
  }

  Eigen::Matrix4f t;
  reg_.estimateRigidTransformation(points, ideal, t);

  // Return board estimate
  board = tfFromEigen(t.inverse());
  return true;
}

bool BoardFinder::accept(cv::Point& p, std::vector<cv::Point>& points)
{
  for (size_t k = 0; k < points.size(); ++k)
  {
    cv::Point tp = points[k];
    if (abs(tp.x - p.x) + abs(tp.y - p.y) < 7)
    {
      return false;
    }
  }
  return true;
}

bool BoardFinder::accept_3d(pcl::PointXYZRGB& p, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  for (size_t k = 0; k < cloud.size(); ++k)
  {
    pcl::PointXYZRGB tp = cloud[k];
    if (fabs(tp.x - p.x) + fabs(tp.y - p.y) + fabs(tp.z - p.z) < 0.05)
    {
      return false;
    }
  }
  return true;
}

cv::Point BoardFinder::findIntersection(cv::Vec4i a, cv::Vec4i b)
{
  /* 5/30/11 - added these checks to avoid problems with vertical lines -- MEF */
  if (a[2] == a[0]) a[2]++;
  if (b[2] == b[0]) b[2]++;

  double ma = (a[3] - a[1]) / static_cast<double>(a[2] - a[0]);
  double mb = (b[3] - b[1]) / static_cast<double>(b[2] - b[0]);
  double ba = a[1] - ma * a[0];
  double bb = b[1] - mb * b[0];

  double x = (bb - ba) / (ma - mb);
  double y = ma * x + ba;

  if ((x >= 0) && (x < 640) && (y >= 0) && (y < 480))
  {
    return cv::Point(static_cast<int>(x), static_cast<int>(y));
  }
  else
  {
    return cv::Point(-1, -1);
  }
}
