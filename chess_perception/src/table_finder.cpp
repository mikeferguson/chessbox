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

#include <chess_perception/table_finder.h>

TableFinder::TableFinder()
{
    ros::NodeHandle nh ("~");

    /* TODO read params from parameter server */
    debug_ = true;
    leaf_size_ = 0.01;

    if(debug_)
    {
        table_cloud_pub_ = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("table", 1);
        hull_cloud_pub_ = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("table_hull", 1);
    }

    /* Setup voxel grid */
    voxel_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_.setFilterLimits(0.25, 1.2);
    voxel_.setFilterFieldName("z");
    
    /* Setup nromal estimation */
    normals_.setRadiusSearch(0.02);

    /* Setup segmentation */
    segmentation_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    segmentation_.setMethodType(pcl::SAC_RANSAC);
    segmentation_.setDistanceThreshold(0.1);
    segmentation_.setMaxIterations(1000);
    segmentation_.setOptimizeCoefficients(true);
    segmentation_.setProbability(0.75);

    /* Setup inlier projection */
    project_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
}

bool TableFinder::findTable(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull)
{
    ROS_DEBUG_STREAM("Table Finder: Input cloud has " << cloud->size() << " points.");

    /* Apply voxel filter */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_.setInputCloud(cloud);
    voxel_.filter(*cloud_downsampled);
    ROS_DEBUG_STREAM("Table Finder: Downsampled cloud has " << cloud_downsampled->size() << " points.");

    /* Estimate point normals */
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    normals_.setInputCloud(cloud_downsampled);
    normals_.setSearchMethod(tree);
    normals_.compute(*cloud_normals);
    ROS_DEBUG_STREAM("Table Finder: Normal cloud has " << cloud_normals->size() << " points.");
    
    /* Segment table plane, extract points */
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    segmentation_.setInputNormals(cloud_normals);
    segmentation_.setInputCloud(cloud_downsampled);
    segmentation_.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0)
    {
        ROS_ERROR("Table Finder: Failed to find a planar model for the table.");
        return false;
    }

    ROS_DEBUG_STREAM("Table Finder: Model Coefficients: " << coefficients->values[0] << " "
                                                         << coefficients->values[1] << " "
                                                         << coefficients->values[2] << " "
                                                         << coefficients->values[3]);
    /* Project inliers */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    project_.setInputCloud(cloud_downsampled);
    project_.setModelCoefficients(coefficients);
    project_.setIndices(inliers);
    project_.filter(*cloud_projected);
    ROS_DEBUG_STREAM("Table Finder: Projected cloud has " << cloud_projected->size() << " points.");

    /* Compute convex hull */
    cloud_hull.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    convexhull_.setDimension(2);
    convexhull_.setInputCloud(cloud_projected);
    convexhull_.reconstruct(*cloud_hull);
    ROS_DEBUG_STREAM("Table Finder: Hull has " << cloud_hull->size() << " points.");

    if(debug_)
    {
        table_cloud_pub_.publish(cloud_projected);
        hull_cloud_pub_.publish(cloud_hull);
    }

    return true;
}
