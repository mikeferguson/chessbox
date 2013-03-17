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

#include <chess_perception/piece_finder.h>
#include <chess_perception/board_finder.h>

#include <chess_msgs/ChessBoard.h>

/** \brief This class handles the estimation, and ties together the other
 *  aspects of board/piece perception.
 */
class ChessPerception
{
  public:
    static const int square_size = 0.05715;

    ChessPerception(ros::NodeHandle & n): nh_ (n)
    {
        debug_ = true;

        /* Subscribe to just the cloud now */
        cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &ChessPerception::cameraCallback, this);
        output_ = nh_.advertise<chess_msgs::ChessBoard>("chess_board_state", 1);
    }

    /** \brief Main loop */
    void cameraCallback ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud )
    {
        /* Find potential corner points of board.
         * This is a mostly 2d-operation that is quite fast, but somewhat unreliable.
         * We do this first, so if it fails we can abort the slower table/piece finding.
         */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points (new pcl::PointCloud<pcl::PointXYZRGB>);
        tf::Transform tr;
        if(!board_finder_.findBoard(cloud, tr))
        {
            ROS_WARN("Unable to detect chess board.");
            return;
        }
        /* Publish board estimate */
        br_.sendTransform(tf::StampedTransform(tr, ros::Time::now(), cloud->header.frame_id, "chess_board"));

        /* Find potential centroids/colors of pieces */
        std::vector<pcl::PointXYZ> pieces;
        std::vector<double> weights;
        int piece_count = piece_finder_.findPieces(cloud, tr, pieces, weights);
        if(piece_count == 0)
        {
            ROS_ERROR("Unable to detect pieces.");
            return;
        }
        ROS_INFO_STREAM("Found " << piece_count << " pieces.");
        /* Publish piece estimate */
        chess_msgs::ChessBoard cb;
        for (size_t i = 0; i < piece_count; i++)
        {
            chess_msgs::ChessPiece p;
            p.header.frame_id = "chess_board";
            p.header.stamp = cloud->header.stamp;
            p.pose.position.x = pieces[i].x;
            p.pose.position.y = pieces[i].y;
            p.pose.position.z = pieces[i].z;
            if(weights[i] > 0)
                p.type = chess_msgs::ChessPiece::WHITE_UNKNOWN;
            else
                p.type = chess_msgs::ChessPiece::BLACK_UNKNOWN;
            cb.pieces.push_back(p);
        }
        output_.publish(cb);
    }

  private:
    /* node handles, subscribers, publishers, etc */
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher output_;
    ros::Publisher projected_points_cloud_pub_;
    tf::TransformBroadcaster br_;
    tf::TransformListener listener_;

    bool debug_;

    /* smarts */
    BoardFinder board_finder_;
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
