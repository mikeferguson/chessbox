/**

Copyright (c) 2011 Michael E. Ferguson.  All right reserved.

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

@b Publish a transform between the checkerboard and the camera link. 

**/

#include <iostream>
#include <algorithm>
#include <limits>
#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <dynamic_reconfigure/server.h>
#include <chess_player/LocatorConfig.h>

using namespace std;
typedef pcl::PointXYZ point;
typedef pcl::PointXYZRGB color_point;

using std::isnan;

/** @brief Helper function to find intersection of two lines */ 
cv::Point findIntersection( cv::Vec4i a, cv::Vec4i b )
{
    /* 5/30/11 - added these checks to avoid problems with vertical lines -- MEF */
    if( a[2] == a[0] ) a[2]++;
    if( b[2] == b[0] ) b[2]++;

    double ma = (a[3]-a[1])/(double)(a[2]-a[0]);
    double mb = (b[3]-b[1])/(double)(b[2]-b[0]);
    double ba = a[1] - ma*a[0];
    double bb = b[1] - mb*b[0];
    
    double x = (bb-ba)/(ma-mb);
    double y = ma*x + ba;

    if( (x>=0) && (x<640) && (y>=0) && (y<480) ){
        return cv::Point((int)x,(int)y);
    }else{
        return cv::Point(-1,-1);
    }
}

/** @brief Helper function to decide whether to accept a point */
bool accept(color_point& cp, pcl::PointCloud<point>::ConstPtr cloud)
{
    if( isnan(cp.x) || isnan(cp.y) || isnan(cp.z) ) return false;
    for(size_t k = 0; k < cloud->points.size(); k++)
    {   
        point tp = cloud->points[k];
        if( abs(tp.x-cp.x) + abs(tp.y-cp.y) + abs(tp.z-cp.z) < 0.05 )
            return false;
    }
    return true;
}

/** @brief Helper function to convert Eigen transformation to tf -- thanks to Garret Gallagher */
tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
    btMatrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
               trans(1,0),trans(1,1),trans(1,2),
               trans(2,0),trans(2,1),trans(2,2));
    btTransform ret;
    ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
}

/** @brief A class for locating the chess board and publishing a transform. 
  * Detection proceeds as follows--
  *   1)  RGB image is converted a grayscale using the X-channel only. 
  *   2)  We threshold the image and then run a Canny edge detector, 
  *         various dilations/erosions are used to improve performance. 
  *   3)  We perform a hough transformation to find lines.
  *   4)  We split lines into horizontal/vertical groups.
  *   5)  We find intersections between lines in the horizontal/vertical groups. 
  *   6)  Each intersection point is then converted to the corresponding point
  *         in the point cloud (x,y,z).  
  *   7) We iterate through possible orientations, finding best fit. 
  */
class ChessBoardLocator
{
  public:
    static const int square_size = 0.05715;

    typedef chess_player::LocatorConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

    ChessBoardLocator(ros::NodeHandle & n): nh_ (n),
        msgs_(0),   
        channel_(0),
        output_image_(false)
    {
        ros::NodeHandle nh ("~");
        // load parameters for hough transform
        if (!nh.getParam ("h_rho", h_rho_))
            h_rho_ = 1;
        ROS_INFO ("Hough Rho: %d", h_rho_);  
        if (!nh.getParam ("h_threshold", h_threshold_))
            h_threshold_ = 50;
        ROS_INFO ("Hough Threshold: %d", h_threshold_); 
        if (!nh.getParam ("h_min_length", h_min_length_))
            h_min_length_ = 100;
        ROS_INFO ("Hough Min Length: %d", h_min_length_);    
        if (!nh.getParam ("e_threshold", e_threshold_))
            e_threshold_ = 0.002;
        ROS_INFO ("Error Threshold: %f", e_threshold_); 

        // publisher for image output
        image_transport::ImageTransport it(nh);
        pub_ = it.advertise("image",1);

        // subscribe to just the cloud now
        cloud_sub_ = nh_.subscribe("/camera/rgb/points", 1, &ChessBoardLocator::cameraCallback, this);
        cloud_pub_ = nh_.advertise< pcl::PointCloud<point> >("cloud", 1);
        
        // initialize dynamic reconfigure
        reconfigure_server_.reset (new ReconfigureServer (reconfigure_mutex_, nh));
        reconfigure_server_->setCallback (boost::bind (&ChessBoardLocator::configCallback, this, _1, _2));
    }

    /* 
     * Determine transform for chess board
     */
    void cameraCallback ( const sensor_msgs::PointCloud2ConstPtr& msg )
    {
        sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);

        // convert cloud to PCL
        pcl::PointCloud<color_point> cloud;
        pcl::fromROSMsg(*msg, cloud);
 
        // get an OpenCV image from the cloud
        pcl::toROSMsg (cloud, *image_msg);

        // convert image to OpenCV 
        try
        {
            bridge_ = cv_bridge::toCvCopy(image_msg, "rgb8");
            ROS_INFO("New image/cloud.");
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("Conversion failed");
        }

        // segment based on a channel (blue board squares)
        cv::Mat dst, cdst;
        cv::Mat src(bridge_->image.rows, bridge_->image.cols, CV_8UC1);
        for(int i = 0; i < bridge_->image.rows; i++)
        {
            char* Di = bridge_->image.ptr<char>(i);
            char* Ii = src.ptr<char>(i);
            for(int j = 0; j < bridge_->image.cols; j++)
            {   
                Ii[j+channel_] = Di[j*3+channel_];
            }   
        }
 
        // threshold, erode/dilate to clean up image
        cv::threshold(src, src, 100, 255, cv::THRESH_BINARY);
        cv::erode(src, src, cv::Mat());
        cv::dilate(src, src, cv::Mat());
        // edge detection, dilation before hough transform 
        cv::Canny(src, dst, 30, 200, 3); 
        cv::dilate(dst, dst, cv::Mat());

        // do a hough transformation to find lines
        vector<cv::Vec4i> lines;
        cv::HoughLinesP(dst, lines, h_rho_, CV_PI/180, h_threshold_, h_min_length_, 10 );
        ROS_DEBUG("Found %d lines", (int) lines.size());

        // split into vertical/horizontal lines
        vector<int> h_indexes, v_indexes;
        for( size_t i = 0; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
            if( l[0] < 100 or l[0] > 500) continue;
            if( l[2] < 100 or l[2] > 500) continue;
            int dx = l[2]-l[0]; int dy = l[3]-l[1];
            if(abs(dx) > abs(dy)){
                h_indexes.push_back(i);
            }else{
                v_indexes.push_back(i);
            }
        }

        // output lines to screen
        if(output_image_)
        {
            // convert back to color
            cv::cvtColor(dst, cdst, CV_GRAY2BGR);
            
            // then red/green for horizontal/vertical
            ROS_DEBUG("horizontal lines: %d", (int) h_indexes.size());
            for( size_t i = 0; i < h_indexes.size(); i++ )
            {
                cv::Vec4i l = lines[h_indexes[i]];
                cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
            }
            ROS_DEBUG("vertical lines: %d", (int) v_indexes.size());
            for( size_t i = 0; i < v_indexes.size(); i++ )
            {
                cv::Vec4i l = lines[v_indexes[i]];
                cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 3, CV_AA);
            }
        }

        // get all intersections
        pcl::PointCloud<point> data;
        data.header.frame_id  = msg->header.frame_id;
        data.header.stamp  = msg->header.stamp;
        for( size_t i = 0; i < h_indexes.size(); i++ )
        {
            cv::Vec4i hl = lines[h_indexes[i]];
            for( size_t j = 0; j < v_indexes.size(); j++ )
            {
                cv::Vec4i vl = lines[v_indexes[j]];
                cv::Point p = findIntersection(hl,vl);
                if(p.x > 0 && p.y > 0){
                    color_point cp = cloud(p.x,p.y);
                    if( accept(cp, data.makeShared()) )
                    {
                        data.push_back( point(cp.x,cp.y,cp.z) );
                        if(output_image_)
                        {
                            cv::circle( cdst, p, 5, cv::Scalar(255,0,0), -1 );
                        }
                    }
                }
            }
        }
        // add endpoints
        for( size_t i = 0; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
            
            cv::Point e1(l[0],l[1]);
            cv::Point e2(l[2],l[3]);

            color_point cp = cloud(e1.x,e1.y);
            if( accept(cp, data.makeShared()) )
            {
                data.push_back( point(cp.x,cp.y,cp.z) );
                if(output_image_)
                {
                    cv::circle( cdst, e1, 5, cv::Scalar(255,0,0), -1 );
                }
            }

            cp = cloud(e2.x,e2.y);
            if( accept(cp, data.makeShared()) )
            {
                data.push_back( point(cp.x,cp.y,cp.z) );
                if(output_image_)
                {
                    cv::circle( cdst, e2, 5, cv::Scalar(255,0,0), -1 );
                }
            }
        }
        ROS_INFO("Created data cloud of size %d", (int)data.points.size());
        if( (int) data.points.size() == 0 ) return;

        // remove outliers
        pcl::PointCloud<point> data_filtered;
        radius_removal_.setMinNeighborsInRadius(2);
        radius_removal_.setRadiusSearch(0.06);
        radius_removal_.setInputCloud(data.makeShared());
        radius_removal_.filter(data_filtered);

        ROS_INFO("Filtered data cloud of size %d", (int)data_filtered.points.size());

        // find centroid of intersections
        Eigen::Vector4f centroid; 
        pcl::compute3DCentroid(data_filtered, centroid);
        ROS_INFO("Centroid (%f, %f)", centroid[0], centroid[1]);

        // find corner candidates - x right, y down
        vector<int> a1_candidates, a8_candidates, h1_candidates;
        for( size_t i = 0; i < data_filtered.points.size(); i++ )
        {
            point p = data_filtered.points[i];
            if( (p.x < centroid[0]-0.05) && (p.y > centroid[1]+0.05) )
                a1_candidates.push_back(i);
            else if( (p.x < centroid[0]-0.05) && (p.y < centroid[1]-0.05) )
                a8_candidates.push_back(i);
            else if( (p.x > centroid[0]+0.05) && (p.y > centroid[1]+0.05) )
                h1_candidates.push_back(i);
        }

        // evaluate candidates
        float best_score = 1000.0;
        int best_points = 0;
        pcl::PointCloud<point> best_cloud;
        Eigen::Matrix4f best_transform;
        ROS_INFO("Evaluating %d candidates (%d, %d, %d)", (int) (a1_candidates.size() * a8_candidates.size() * h1_candidates.size()), (int) a1_candidates.size(), (int) a8_candidates.size(), (int) h1_candidates.size());

        for( size_t iter = 1; iter < 4; iter++){
            // ideal board of a1, a8, h1 (then b2, b7, g2, etc)
            pcl::PointCloud<point> board;
            board.push_back( point(0.05715*iter, 0.05715*iter, 0) );     // a1
            board.push_back( point(0.05715*iter, 0.05715*(8-iter), 0) ); // a8
            board.push_back( point(0.05715*(8-iter), 0.05715*iter, 0) ); // h1

            for( size_t i = 0; i < a1_candidates.size(); i++ ){
                for( size_t j = 0; j < a8_candidates.size(); j++ ){
                    for( size_t k = 0; k < h1_candidates.size(); k++ ){
                        Eigen::Matrix4f t;
                        point a1 = data_filtered.points[a1_candidates[i]];
                        point a8 = data_filtered.points[a8_candidates[j]];
                        point h1 = data_filtered.points[h1_candidates[k]];
                        // check reasonableness
                        if( (abs(pcl::euclideanDistance(a1,a8) - (0.05715*(8-iter*2))) > 0.02 ) ||
                            (abs(pcl::euclideanDistance(a1,h1) - (0.05715*(8-iter*2))) > 0.02 ) ||
                            (abs(pcl::euclideanDistance(h1,a8) - (0.05715*(8-iter*2)*1.4142)) > 0.02 ) ) continue;
                        // construct a basis
                        pcl::PointCloud<point> candidates;
                        candidates.push_back(a1);
                        candidates.push_back(a8); 
                        candidates.push_back(h1);
                        // estimate transform
                        pcl::estimateRigidTransformationSVD( candidates, board, t );
                        // transform whole cloud
                        pcl::PointCloud<point> data_transformed;
                        pcl::transformPointCloud( data_filtered, data_transformed, t );
                        // compute error
                        float error = 0.0;
                        int points = 0;
                        for( size_t p = 0; p < data_transformed.points.size(); p++)
                        {
                            point pt = data_transformed.points[p];
                            // TODO: can we speed this up?
                            float e = 1000;
                            for( int x = 0; x <= 8; x++)
                            {   
                                for( int y = 0; y <= 8; y++)
                                {
                                    float test = (0.05715*x-pt.x)*(0.05715*x-pt.x)+(0.05715*y-pt.y)*(0.05715*y-pt.y);
                                    if(test < e)
                                        e = test;
                                }
                            }
                            if(e < 0.05){
                                error += e; 
                                if( e < 0.02)
                                    points++;
                            } // else outlier
                        }
                        // update
                        if( ((points > (data_transformed.points.size()*3)/4) || (points > 60)) && (error < best_score) )
                        {
                            best_score = error; 
                            best_points = points;
                            best_transform = t;
                            best_cloud = data_transformed;
                        }                    
                    }
                }
            }    
            ROS_INFO("best score %f", best_score);
            ROS_INFO("best points %d", best_points);   
            // error is under threshold, we are likely done
            if(best_score < e_threshold_) break;
        }

        best_cloud.header.frame_id = "chess_board_raw";
        cloud_pub_.publish( best_cloud );
          
        // publish transform
        tf::Transform transform = tfFromEigen(best_transform.inverse());
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), msg->header.frame_id, "chess_board_raw"));
        ROS_INFO("published %d", msgs_++);

        if(output_image_){
            bridge_->image = cdst;
            pub_.publish( bridge_->toImageMsg() );
        }
    }

    void configCallback (Config &config, uint32_t level)
    {
        h_rho_ = config.h_rho;
        h_threshold_ = config.h_threshold;
        h_min_length_ = config.h_min_length;
        channel_ = config.channel;
        output_image_ = config.output_image;
    }


  private: 
    /* node handles, subscribers, publishers, etc */
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    image_transport::Publisher pub_;
    tf::TransformBroadcaster br_;
    cv_bridge::CvImagePtr bridge_;
    pcl::RadiusOutlierRemoval<point> radius_removal_;

    /* reconfigure server*/
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;
    boost::recursive_mutex reconfigure_mutex_;

    /* parameters for hough line detection */
    int h_rho_;
    int h_threshold_;
    int h_min_length_;
    double e_threshold_;
    int msgs_;
    int channel_;
    bool output_image_;
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "chess_board_locator");
  ros::NodeHandle n;
  ChessBoardLocator locator(n);
  ros::spin();
  return 0;
}

