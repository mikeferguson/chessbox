/**

Copyright (c) 2011-2024 Michael E. Ferguson.  All right reserved.

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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <chess_perception/piece_finder.h>
#include <chess_perception/board_finder.h>

#include <chess_msgs/msg/chess_board.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("chess_perception");

/**
 * @brief This class handles the estimation, and ties together the other
 *        aspects of board/piece perception.
 */
class ChessPerception
{
public:
  ChessPerception(rclcpp::Node::SharedPtr node): node_(node), board_finder_(node), piece_finder_(node)
  {
    // Initialize cached data to something reasonable
    board_to_fixed_.setIdentity();
    frames_ = 0;
    debug_ = true;

    // Setup TF
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);

    // Load parameters
    skip_ = node_->declare_parameter<int>("skip", 2);
    fixed_frame_ = node_->declare_parameter<std::string>("fixed_frame", "base_link");
    double square_size = node_->declare_parameter<double>("chess_square_size", 0.5715);
    board_finder_.setSquareSize(square_size);
    piece_finder_.setSquareSize(square_size);

    // Subscribe to just the cloud now
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/head_camera/depth_registered/points",
      rclcpp::QoS(1).best_effort(),
      std::bind(&ChessPerception::cameraCallback, this, std::placeholders::_1));

    // Publish board output
    output_ = node_->create_publisher<chess_msgs::msg::ChessBoard>("chess_board_state", 1);

    // Periodic callback to publish tf
    publish_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&ChessPerception::publishCallback, this));
  }

  /** @brief Main loop */
  void cameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if ((frames_++ % skip_) != 0)
    {
      return;
    }

    // Get transform from camera->fixed
    tf2::Transform tr2;
    try
    {
      auto t = buffer_->lookupTransform(fixed_frame_, msg->header.frame_id, tf2::TimePointZero);
      tf2::fromMsg(t.transform, tr2);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(LOGGER, "%s", ex.what());
      return;
    }

    // Convert to point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*msg, *cloud);

    /*
     * Find potential corner points of board.
     * This is a mostly 2d-operation that is quite fast, but somewhat unreliable.
     * We do this first, so if it fails we can abort the slower table/piece finding.
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    tf2::Transform tr;
    if (!board_finder_.findBoard(cloud, tr))
    {
      RCLCPP_WARN(LOGGER, "Unable to detect chess board.");
      return;
    }

    // Update board estimate
    board_to_fixed_ = tr2 * tr;

    // Find potential centroids/colors of pieces
    std::vector<pcl::PointXYZ> pieces;
    std::vector<double> weights;
    size_t piece_count = piece_finder_.findPieces(cloud, tr, pieces, weights);
    if (piece_count == 0)
    {
      RCLCPP_WARN(LOGGER, "Unable to detect pieces.");
      return;
    }
    RCLCPP_DEBUG(LOGGER, "Found %lu pieces.", piece_count);

    // Publish piece and transform estimate
    chess_msgs::msg::ChessBoard cb;
    for (size_t i = 0; i < piece_count; ++i)
    {
      chess_msgs::msg::ChessPiece p;
      p.header = msg->header;
      p.header.frame_id = "chess_board";
      p.pose.position.x = pieces[i].x;
      p.pose.position.y = pieces[i].y;
      p.pose.position.z = pieces[i].z;
      if (weights[i] > 0)
      {
        p.type = chess_msgs::msg::ChessPiece::WHITE_UNKNOWN;
      }
      else
      {
        p.type = chess_msgs::msg::ChessPiece::BLACK_UNKNOWN;
      }
      cb.pieces.push_back(p);
    }
    cb.board_to_fixed.header.frame_id = fixed_frame_;
    cb.board_to_fixed.header.stamp = node_->now();
    cb.board_to_fixed.child_frame_id = "chess_board";
    cb.board_to_fixed.transform = tf2::toMsg(board_to_fixed_);
    output_->publish(cb);
  }

  /** @brief Periodic callback to publish tf data */
  void publishCallback()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = fixed_frame_;
    transform.child_frame_id = "chess_board_estimate";
    transform.transform = tf2::toMsg(board_to_fixed_);
    br_->sendTransform(transform);
  }

private:
  // Node handles, subscribers, publishers, etc
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<chess_msgs::msg::ChessBoard>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;

  // The frame to cache and republish in (typically "base_link")
  std::string fixed_frame_;
  // The actual cached transform to publish
  tf2::Transform board_to_fixed_;

  int skip_;
  unsigned int frames_;
  bool debug_;

  // Smarts
  BoardFinder board_finder_;
  PieceFinder piece_finder_;
};

int main (int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("chess_perception_node");
  ChessPerception perception(node);
  rclcpp::spin(node);
  return 0;
}
