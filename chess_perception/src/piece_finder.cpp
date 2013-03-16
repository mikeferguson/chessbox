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

PieceFinder::PieceFinder()
{
    ros::NodeHandle nh ("~");

    /* TODO read params from parameter server */
    debug_ = true;

    if(debug_)
    {
        pieces_cloud_pub_ = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("pieces", 1);
    }

    /* Setup extract polygonal prism data */
    extract_data_.setHeightLimits(0.01, 0.2);

    /* Setup extracting indices */
    extract_indices_.setNegative(false);

    /* Setup clustering */
    cluster_.setClusterTolerance(0.01);
    cluster_.setMinClusterSize(10);
    cluster_.setMaxClusterSize(3000); // TODO: is this too low?
}

int PieceFinder::findPieces(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr table_hull,
                            std::vector<pcl::PointXYZ>& pieces,
                            std::vector<double>& weights)
{
    /* Extract points above convex hull */
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    extract_data_.setInputCloud(cloud);
    extract_data_.setInputPlanarHull(table_hull);
    extract_data_.segment(*inliers);

    /* Extract points from indices found above */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pieces (new pcl::PointCloud<pcl::PointXYZRGB>);
    extract_indices_.setInputCloud(cloud);
    extract_indices_.setIndices(inliers);
    extract_indices_.filter(*cloud_pieces);
    ROS_DEBUG_STREAM("Piece Finder: Extract points has " << cloud_pieces->size() << " points.");

    if(debug_)
        pieces_cloud_pub_.publish(cloud_pieces);

    /* Cluster */
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    std::vector<pcl::PointIndices> clusters;
    cluster_.setSearchMethod(tree);
    cluster_.setInputCloud(cloud_pieces);
    cluster_.extract(clusters);
    ROS_DEBUG_STREAM("Piece Finder: Found " << clusters.size() << " clusters.");

    /* Determine Weights */
    for (size_t c = 0; c < clusters.size(); ++c)
    {
        /* find color/centroid of this cluster */
        double x = 0; double y = 0; double z = 0; int color = 0;
        for (size_t i = 0; i < clusters[c].indices.size(); i++)
        {
            int j = clusters[c].indices[i];
            x += cloud_pieces->points[j].x;
            y += cloud_pieces->points[j].y;
            z += cloud_pieces->points[j].z;
            unsigned char * rgb = (unsigned char *) &(cloud_pieces->points[j].rgb);
            color += (rgb[0] + rgb[1] + rgb[2])/3;
        }
        x = x/clusters[c].indices.size();
        y = y/clusters[c].indices.size();
        z = z/clusters[c].indices.size();
        color = color/clusters[c].indices.size();
        
        /* outputs */
        pcl::PointXYZ p(x,y,z);
        double weight = clusters[c].indices.size();
        if(color < 100)
            weight = -weight; // use negative numbers for black

        /* check if cluster overlaps any other */
        bool new_ = true;
        for (size_t j = 0; j < pieces.size(); j++)
        {
            if( (fabs(p.x-pieces[j].x) < 0.012) &&
                (fabs(p.y-pieces[j].y) < 0.012) )
            {
                double w1 = fabs(weights[j]);
                double w2 = fabs(weight);
                pieces[j].x = (pieces[j].x*w1 + p.x*w2)/(w1+w2);
                pieces[j].y = (pieces[j].y*w1 + p.y*w2)/(w1+w2);
                pieces[j].z = (pieces[j].z*w1 + p.z*w2)/(w1+w2);
                weights[j] += weight;
                new_ = false;
                break;
            }
        }
    
        /* add cluster if not overlapping with another */
        if (new_){
            // TODO: add check for too large to be a chess piece
            pieces.push_back(p);
            weights.push_back(weight);
        }
    }

    return weights.size();
}
