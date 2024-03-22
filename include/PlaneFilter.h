/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-04-10 14:53
#
# Filename:		PlaneFilter.h
#
# Description: 
#
===============================================*/
#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <eigen3/Eigen/Core>
#include "Basics.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace sun
{
	class PlaneFilter
	{
	public:
		PlaneFilter();
		~PlaneFilter() {}

		void setDebug(bool d) {debug=d;}

		// loadScan
		// - img_rgb,img_depth;
		// - point_cloud, normal_cloud, pixel_cloud;
		// - planes
		//   - points_in,num_points;
		//   - normal,d,avg_rgb,avg_pps,cov_rgb,cov_pps;
		void loadScan(Scan *s) {scan=s;}

		void filter();
		void filter(Plane *plane);

		void EdgeFilter();
		void ColorFilter();
		void DensityFilter();
	private:
		// scan
		// - loaded with
		//   - img_rgb,img_depth;
		//   - point_cloud, normal_cloud, pixel_cloud;
		//   - planes
		//     - points_in,num_points;
		//     - normal,d,avg_rgb,avg_pps,cov_rgb,cov_pps;
		Scan *scan;

		// index_map
		// - map to each point on a plane;
		// - indexed by its pixel coordinate;
		std::map<Pixel,Point*> index_map;

		// the response of the color filter;
		std::map<Pixel,double> color_response;
		Eigen::Matrix3d temp_color;
		// the response of the edge filter;
		std::map<Pixel,double> edge_response;
		Eigen::Matrix3d temp_edge;
		// the grey value of each point on plane;
		std::map<Pixel,double> grey;
		// point density at each point on plane;
		std::map<Pixel,double> density_response;
		Eigen::Matrix3d temp_density;
		double cx,cy;
		bool debug;
		void loadPlane(Plane *plane);
		Eigen::Matrix3d rotation_plane2camera(Eigen::Vector3d n);
		double ColorResponse(Pixel index, Eigen::MatrixXd temp_color);
		double EdgeResponse(Pixel index, Eigen::MatrixXd temp_edge);
		double DensityResponse(Pixel index, Eigen::MatrixXd temp_density);
		double DepthWeight(Pixel index);
		double CenterWeight(Pixel index);
		void color2grey();
	};
}
