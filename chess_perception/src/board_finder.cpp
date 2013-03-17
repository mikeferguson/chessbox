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

#include <chess_perception/board_finder.h>

#include <ros/ros.h>

/** Sort points based only on x-component */
bool orderPointsX(const cv::Point& a, const cv::Point& b)
{
    return a.x < b.x;
}

/** Order horizontal lines (bottom of screen first) */
bool orderLinesH(const cv::Vec4i& a, const cv::Vec4i& b)
{
    double ma = (a[3]-a[1])/(double)(a[2]-a[0]);
    double mb = (b[3]-b[1])/(double)(b[2]-b[0]);
    double ba = a[1] - ma*a[0];
    double bb = b[1] - mb*b[0];
    return ba > bb;
}

BoardFinder::BoardFinder()
{
    /* TODO: load these from parameters */

    ros::NodeHandle nh ("~");

    channel_ = 0; // blue

    /* Load parameters for hough transform */
    if (!nh.getParam ("h_rho", h_rho_))
        h_rho_ = 1;
    ROS_INFO ("Hough Rho: %d", h_rho_);
    if (!nh.getParam ("h_threshold", h_threshold_))
        h_threshold_ = 50;
    ROS_INFO ("Hough Threshold: %d", h_threshold_);
    if (!nh.getParam ("h_min_length", h_min_length_))
        h_min_length_ = 100;
    ROS_INFO ("Hough Min Length: %d", h_min_length_);

    point_threshold_ = 43;

    debug_ = true;

    /* Debug image output */
    if(debug_)
    {
        image_transport::ImageTransport it(nh);
        pub_ = it.advertise("board_finder_image",1);
    }
}

bool BoardFinder::findCorners(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                              pcl::PointCloud<pcl::PointXYZRGB>& points)
{
    /* Copy header to preserve frame */
    points.header = cloud->header;

    /* Get an OpenCV image from the cloud 
           TODO: update this code to be better/faster... */
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*cloud, *image_msg);
    try
    {
        bridge_ = cv_bridge::toCvCopy(image_msg, "rgb8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Conversion failed");
    }

    /* Segment based on a channel (blue board squares) */
    cv::Mat dst, cdst;
    cv::Mat src(bridge_->image.rows, bridge_->image.cols, CV_8UC1);
    for(int i = 0; i < bridge_->image.rows; i++)
    {
        const char* Di = bridge_->image.ptr<char>(i);
        char* Ii = src.ptr<char>(i);
        for(int j = 0; j < bridge_->image.cols; j++)
        {   
            Ii[j+channel_] = Di[j*3+channel_];
        }   
    }

    /* Threshold, erode/dilate to clean up image */
    cv::threshold(src, src, 120, 255, cv::THRESH_BINARY);
    cv::erode(src, src, cv::Mat());
    cv::dilate(src, src, cv::Mat());
    /* Edge detection, dilation before hough transform */
    cv::Canny(src, dst, 30, 200, 3); 
    cv::dilate(dst, dst, cv::Mat());

    /* Do a hough transformation to find lines */
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dst, lines, h_rho_, CV_PI/180, h_threshold_, h_min_length_, 10 );
    ROS_DEBUG("Found %d lines", (int) lines.size());

    /* Split into vertical/horizontal lines */
    std::vector<cv::Vec4i> h_lines, v_lines;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i a = lines[i];
        int dx = a[2]-a[0]; int dy = a[3]-a[1];
        if(abs(dx) > abs(dy)){
            bool push = true;
            double ma = (a[3]-a[1])/(double)(a[2]-a[0]);
            double ba = a[1] - ma*a[0];
            for( size_t j = 0; j < h_lines.size(); j++)
            {
                cv::Vec4i b = h_lines[j];
                double mb = (b[3]-b[1])/(double)(b[2]-b[0]);
                double bb = b[1] - mb*b[0];
                if( (fabs(ba-bb) < 15.0f) && (fabs(ma-mb) < 0.5) )
                {
                    push = false;
                    break;
                }
            }
            if(push)
                h_lines.push_back(a);
        }else{
            bool push = true;
            double ma = (a[2]-a[0])/(double)(a[3]-a[1]); // x = my+b
            double ba = a[0] - ma*a[1];
            for( size_t j = 0; j < v_lines.size(); j++)
            {
                cv::Vec4i b = v_lines[j];
                double mb = (b[2]-b[0])/(double)(b[3]-b[1]);
                double bb = b[0] - mb*b[1];
                if( (fabs(ba-bb) < 15.0f) && (fabs(ma-mb) < 0.5) )
                {
                    push = false;
                    break;
                }
            }
            if(push)
                v_lines.push_back(a);
        }
    }

    /* Sort horizontal lines */
    std::sort(h_lines.begin(), h_lines.end(), orderLinesH);

    /* Output lines to screen */
    if(debug_)
    {
        /* Convert back to color */
        cv::cvtColor(src, cdst, CV_GRAY2BGR);
            
        /* Then blue/green for horizontal/vertical */
        ROS_DEBUG("horizontal lines: %d", (int) h_lines.size());
        for( size_t i = 0; i < h_lines.size(); i++ )
        {
            cv::Vec4i l = h_lines[i];
            if(i%2 == 0)
                cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 2, CV_AA);
            else
                cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,255), 2, CV_AA);
        }
        ROS_DEBUG("vertical lines: %d", (int) v_lines.size());
        for( size_t i = 0; i < v_lines.size(); i++ )
        {
            cv::Vec4i l = v_lines[i];
            cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 2, CV_AA);
        }
    }

    /* Get all (2d) intersections, which are proper corners */ 
    std::vector<cv::Point> corner_points_2d;
    for( size_t i = 0; i < h_lines.size(); i++ )
    {
        cv::Vec4i hl = h_lines[i];
        std::vector<cv::Point> temp_lines;
        for( size_t j = 0; j < v_lines.size(); j++ )
        {
            /* For each vertical and horizontal line pair, find the intersection */
            cv::Vec4i vl = v_lines[j];
            cv::Point p = findIntersection(hl,vl);
            /* Test that intersection exists, and is not close to a point already picked */
            if(p.x > 0 && p.y > 0)
            {
                /* Test if this really is a checkerboard intersection by checking
                 * the color of points on four sides.
                 * TODO: this is a mockup, add a real check
                 */
                if(src.at<unsigned char>(p.y + 7, p.x + 7) < 120)
                {
                    if(src.at<unsigned char>(p.y + 7, p.x - 7) < 120 ||
                       src.at<unsigned char>(p.y - 7, p.x - 7) > 120 ||
                       src.at<unsigned char>(p.y - 7, p.x + 7) < 120)
                    {
                        if(debug_)
                            cv::circle( cdst, p, 5, cv::Scalar(255,255,0), -1 );
                        continue;
                    }
                }
                else
                {
                    if(src.at<unsigned char>(p.y + 7, p.x - 7) > 120 ||
                       src.at<unsigned char>(p.y - 7, p.x - 7) < 120 ||
                       src.at<unsigned char>(p.y - 7, p.x + 7) > 120)
                    {
                        if(debug_)
                            cv::circle( cdst, p, 5, cv::Scalar(255,255,0), -1 );
                        continue;
                    }
                }
                temp_lines.push_back(p);
                if(debug_)
                {
                    /* Draw circles */
                    cv::circle( cdst, p, 5, cv::Scalar(255,0,0), -1 );
                    //ROS_INFO_STREAM(p.x << "," << p.y);
                }
            }
        }
        if(temp_lines.size() > 5)
        {
            std::stable_sort(temp_lines.begin(), temp_lines.end(), orderPointsX);
            for(size_t k = 0; k < temp_lines.size(); k++)
            {
                corner_points_2d.push_back(temp_lines[k]);
            }
        }
    }

    ROS_DEBUG_STREAM("Board Finder: Found " << corner_points_2d.size() << " 2d points");

    if(corner_points_2d.size() > point_threshold_)
    {
        /* Project to 3d */
        for ( size_t i = 0; i < corner_points_2d.size(); i++ )
        {
            cv::Point p2d = corner_points_2d[i];
            pcl::PointXYZRGB p3d = (*cloud)(p2d.x, p2d.y);
            if( !isnan(p3d.x) && !isnan(p3d.y) && !isnan(p3d.z) )
            {
                if( accept_3d(p3d, points) )
                {
                    points.push_back( p3d );
                }
            }
            // TODO: add some means of searching for a point near here that is not a nan.
        }

        ROS_DEBUG_STREAM("Board Finder: Found " << points.size() << " 3d points");
    
        if(points.size() < point_threshold_)
            return false;

        if(debug_)
        {
            bridge_->image = cdst;
            pub_.publish( bridge_->toImageMsg() );
        }
        return true;
    }
    return false;
}

/* Helper function to decide whether to accept a point */
bool BoardFinder::accept(cv::Point& p, std::vector<cv::Point>& points)
{
    for(size_t k = 0; k < points.size(); k++)
    {   
        cv::Point tp = points[k];
        if( abs(tp.x-p.x) + abs(tp.y-p.y) < 7 )
            return false;
    }
    return true;
}

/* Helper function to decide whether to accept a point */
bool BoardFinder::accept_3d(pcl::PointXYZRGB& p, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    for(size_t k = 0; k < cloud.size(); k++)
    {   
        pcl::PointXYZRGB tp = cloud[k];
        if( fabs(tp.x-p.x) + fabs(tp.y-p.y) + fabs(tp.z-p.z) < 0.05 )
            return false;
    }
    return true;
}

/* Helper function to find intersection of two lines */ 
cv::Point BoardFinder::findIntersection( cv::Vec4i a, cv::Vec4i b )
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
