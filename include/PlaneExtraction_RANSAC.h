/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-04-09 14:34
#
# Filename:		PlaneExtraction_RANSAC.h
#
# Description: 
#
===============================================*/
#pragma once
//#define _USE_MATH_DEFINES
#include <stdio.h>
#include <fstream>
#include <vector>
#include <stack>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include "PlaneExtraction.h"
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/sample_consensus/method_types.h>
#include <pcl-1.8/pcl/sample_consensus/model_types.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <algorithm>

namespace sun
{

	class PlaneExtraction_RANSAC : public PlaneExtraction
	{
	public:
		PlaneExtraction_RANSAC();
		~PlaneExtraction_RANSAC() {}

		// ExtractPlanes
		// - extract planes from the scan;
		virtual void ExtractPlanes();

	private:
		double maxdist_point2plane;
		int max_plane;
		int min_plane_size;
		double thres_angle, thres_dist, thres_color;

		// unifyPlaneDir
		// - make the plane normal point to the origin;
		void unifyPlaneDir(pcl::ModelCoefficients::Ptr plane);

		// dist_point2plane
		// - compute the vertical distance from a point to a plane;
		double dist_point2plane(Eigen::Vector3d point, pcl::ModelCoefficients::Ptr plane);

		// computePlaneAvgCov
		// - compute the avg_pps, cov_pps, avg_rgb and cov_rgb from the points_in;
		void computePlaneAvgCov(Plane *plane);

		// fusePlanes
		// - fuse tmp_plane to planes[i_plane];
		void fusePlanes(int i_plane, Plane *cur);
	};
}
