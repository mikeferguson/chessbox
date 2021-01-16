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

#include <chess_perception/piece_finder.h>

// see
// http://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
// for points on a dark background you want somewhat lightened
// colors generally... back off the saturation (s)
static void hsv2rgb(float h, float s, float v, float& r, float& g, float& b)
{
  float c = v * s;
  float hprime = h/60.0;
  float x = c * (1.0 - fabs(fmodf(hprime, 2.0f) - 1));

  r = g = b = 0;

  if (hprime < 1)
  {
    r = c; g = x;
  }
  else if (hprime < 2)
  {
    r = x; g = c;
  }
  else if (hprime < 3)
  {
    g = c; b = x;
  }
  else if (hprime < 4)
  {
    g = x; b = c;
  }
  else if (hprime < 5)
  {
    r = x; b = c;
  }
  else if (hprime < 6)
  {
    r = c; b = x;
  }

  float m = v - c;
  r += m; g+=m; b+=m;
}

PieceFinder::PieceFinder() : square_size_(0.05715)
{
  ros::NodeHandle nh ("~");

  debug_ = true;

  if (!nh.getParam("color_threshold", threshold_))
  {
    threshold_ = 70;
  }

  if (debug_)
  {
    pieces_cloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("pieces", 1);
  }

  setupUntransformedHull();

  // Setup extract polygonal prism data
  extract_data_.setHeightLimits(0.01, 0.2);

  // Setup extracting indices
  extract_indices_.setNegative(false);

  // Setup clustering
  cluster_.setClusterTolerance(0.01);
  cluster_.setMinClusterSize(1);
  cluster_.setMaxClusterSize(3000);
}

void PieceFinder::setupUntransformedHull()
{
  // Remove an points
  hull_untransformed_.clear();

  // Push back the 3 corners
  pcl::PointXYZRGB p;
  hull_untransformed_.push_back(p);
  p.x = square_size_ * 8;
  hull_untransformed_.push_back(p);
  p.y = square_size_ * 8;
  hull_untransformed_.push_back(p);
  p.x = 0;
  hull_untransformed_.push_back(p);
}

void PieceFinder::setSquareSize(double size)
{
  square_size_ = size;
  setupUntransformedHull();
}

size_t PieceFinder::findPieces(
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
  tf::Transform& board_transform,
  std::vector<pcl::PointXYZ>& pieces,
  std::vector<double>& weights)
{
  // Generate a convex hull that is only over the board
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_ros::transformPointCloud(hull_untransformed_, *table_hull, board_transform);
  table_hull->header = cloud->header;

  // Extract points above convex hull
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  extract_data_.setInputCloud(cloud);
  extract_data_.setInputPlanarHull(table_hull);
  extract_data_.segment(*inliers);

  // Extract points from indices found above
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pieces(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract_indices_.setInputCloud(cloud);
  extract_indices_.setIndices(inliers);
  extract_indices_.filter(*cloud_pieces);
  ROS_DEBUG_STREAM("Piece Finder: Extract points has " << cloud_pieces->size() << " points.");

  // Transform points into chess board frame
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_ros::transformPointCloud(*cloud_pieces, *cloud_transformed, board_transform.inverse());
  cloud_transformed->header = cloud->header;
  cloud_transformed->header.frame_id = "chess_board";

  // Cluster
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  std::vector<pcl::PointIndices> clusters;
  cluster_.setSearchMethod(tree);
  cluster_.setInputCloud(cloud_transformed);
  cluster_.extract(clusters);

  ROS_DEBUG_STREAM("Piece Finder: Extracted " << clusters.size() << " clusters.");

  /* Merge clusters vertically, determine centroids and weights */
  std::vector<double> cluster_min_x;
  std::vector<double> cluster_max_x;
  std::vector<double> cluster_min_y;
  std::vector<double> cluster_max_y;
  std::vector< std::vector<int> > cluster_index;
  for (size_t i = 0; i < 64; ++i)
  {
    cluster_index.push_back(std::vector<int>());
  }

  for (size_t c = 0; c < clusters.size(); ++c)
  {
    // Find the limits, and color of this cluster
    double min_x = 1000.0;
    double max_x = -1000.0;
    double min_y = 1000.0;
    double max_y = -1000.0;
    int color = 0;
    for (size_t i = 0; i < clusters[c].indices.size(); ++i)
    {
      int j = clusters[c].indices[i];
      min_x = std::min(static_cast<double>(cloud_transformed->points[j].x), min_x);
      max_x = std::max(static_cast<double>(cloud_transformed->points[j].x), max_x);
      min_y = std::min(static_cast<double>(cloud_transformed->points[j].y), min_y);
      max_y = std::max(static_cast<double>(cloud_transformed->points[j].y), max_y);
      unsigned char * rgb = (unsigned char *) &(cloud_transformed->points[j].rgb);
      color += (rgb[0] + rgb[1] + rgb[2]) / 3.0;
    }
    cluster_min_x.push_back(min_x);
    cluster_max_x.push_back(max_x);
    cluster_min_y.push_back(min_y);
    cluster_max_y.push_back(max_y);
    color = color / clusters[c].indices.size();

    // Outputs
    double x = (max_x + min_x) / 2.0;
    double y = (max_y + min_y) / 2.0;
    pcl::PointXYZ p(x, y, 0.0);
    double weight = clusters[c].indices.size();
    if (color < threshold_)
    {
      weight = -weight;  // use negative numbers for black
    }

    // Check if cluster overlaps any other
    size_t index = (int)(x / square_size_) + 8 * (int)(y / square_size_);
    if (index >= cluster_index.size())
    {
      continue;
    }
    if (cluster_index[index].empty())
    {
      // Is a new cluster
      pieces.push_back(p);
      weights.push_back(weight);
      cluster_index[index].push_back(pieces.size()-1);  // first index is piece, remaining are clusters
      cluster_index[index].push_back(c);
    }
    else
    {
      int p_= cluster_index[index][0];  // first index is piece
      int i_= cluster_index[index][1];  // second index is first cluster, which we update

      // Update limits
      cluster_min_x[i_] = std::min(cluster_min_x[i_], min_x);
      cluster_max_x[i_] = std::max(cluster_max_x[i_], max_x);
      cluster_min_y[i_] = std::min(cluster_min_y[i_], min_y);
      cluster_max_y[i_] = std::max(cluster_max_y[i_], max_y);

      weights[p_] = weights[p_] + weight;

      pieces[p_].x = (cluster_max_x[i_] + cluster_min_x[i_]) / 2.0;
      pieces[p_].y = (cluster_max_y[i_] + cluster_min_y[i_]) / 2.0;
      pieces[p_].z = 0.0;
      cluster_index[index].push_back(c);
    }
  }

  ROS_DEBUG_STREAM("Piece Finder: Found " << weights.size() << " clusters.");

  if (debug_)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
    cluster_cloud.header = cloud_transformed->header;
    for (size_t i = 0; i < 64; i++)
    {
      if (cluster_index[i].size() == 0)
      {
        continue;
      }

      // Determine a new color
      float hue = (360.0 / 20) * (i % 20);
      float r, g, b;
      hsv2rgb(hue, 0.8, 1.0, r, g, b);

      uint32_t rgb = ((uint32_t)(r * 255) << 16 |
                      (uint32_t)(g * 255) << 8  |
                      (uint32_t)(b * 255));

      // Color the cloud
      for (size_t j = 1; j < cluster_index[i].size(); ++j)
      {
        std::vector<int> indices = clusters[cluster_index[i][j]].indices;
        for (size_t k = 0; k < indices.size(); ++k)
        {
          pcl::PointXYZRGB p = (*cloud_transformed)[indices[k]];
          memcpy(&rgb, &p.rgb, 4);
          cluster_cloud.push_back(p);
        }
      }
    }
    pieces_cloud_pub_.publish(cluster_cloud);
  }

  return weights.size();
}
