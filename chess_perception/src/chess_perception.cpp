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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <chess_perception/table_finder.h>
#include <chess_perception/piece_finder.h>
#include <chess_perception/board_finder.h>

/** \brief This class handles the estimation, and ties together the other
 *  aspects of board/piece perception.
 */
class ChessPerception
{
  public:
    static const int square_size = 0.05715;

    ChessPerception(ros::NodeHandle & n): nh_ (n), msgs_(0)
    {
        debug_ = true;
        skip_ = 1;

        /* Subscribe to just the cloud now */
        cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &ChessPerception::cameraCallback, this);
    }

    /** \brief Main loop */
    void cameraCallback ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud )
    {
        /* Transform cloud into base_link
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl_ros::transformPointCloud("base_link", *cloud, *cloud_transformed, listener_);*/

        /* Find potential corner points of board.
         * This is a mostly 2d-operation that is quite fast, but somewhat unreliable.
         * We do this first, so if it fails we can abort the slower table/piece finding.
         */
        boost::shared_ptr< std::vector<pcl::PointXYZ> > points;
        if(!board_finder_.findCorners(cloud, points))
        {
            ROS_WARN("Unable to detect chess board corners.");
            return;
        }

        /* Find the convex hull of the table */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_convex_hull;
        if(!table_finder_.findTable(cloud, table_convex_hull))
        {
            ROS_ERROR("Unable to detect table.");
            return;
        }
        
        /* Find potential centroids/colors of pieces */
        boost::shared_ptr< std::vector<pcl::PointXYZ> > pieces;
        boost::shared_ptr< std::vector<double> > weights;
        int piece_count = piece_finder_.findPieces(cloud, table_convex_hull, pieces, weights);
        if(piece_count == 0)
        {
            ROS_ERROR("Unable to detect pieces.");
            return;
        }

        /* Project to plane */

        /* estimate board/piece pose */

        /* publish board estimate */
    }

  private:
    /* node handles, subscribers, publishers, etc */
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    tf::TransformBroadcaster br_;
    tf::TransformListener listener_;

    /* count of how many messages have been received */
    unsigned int msgs_;
    int skip_;
    bool debug_;

    /* smarts */
    BoardFinder board_finder_;
    TableFinder table_finder_;
    PieceFinder piece_finder_;
};


int main (int argc, char **argv)
{
  ros::init(argc, argv, "chess_perception_node");
  ros::NodeHandle n;
  ChessPerception perception(n);
  ros::spin();
  return 0;
}
