/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-04-01 21:18
#
# Filename:		DataReading.h
#
# Description: 
#
===============================================*/
#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <pcl-1.8/pcl/io/io.h>
#include <pcl-1.8/pcl/io/file_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <eigen3/Eigen/src/Core/DenseBase.h>
#include <pcl-1.8/pcl/features/integral_image_normal.h>
#include <pcl-1.8/pcl/features/normal_3d.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include "Basics.h"

// + foldername
//   + rgb
//   + depth
//   - rgb.txt
//   - depth.txt
//
namespace sun
{
	class DataReading
	{
	public:

		// DataReading.
		// + input
		//   - path_name: path of the folder locating the rgb.txt, depth.txt, rgb folder and depth folder, etc.
		// + function
		//   - assignment of camera intrinsic parameters;
		//   - allocate the point_cloud;
		//   - set default StorageType as Time_Name;
		//   - set default debug as false;
		DataReading(const std::string f);

		~DataReading() {}

		void setDebug(bool d) {debug=d;}

		// setSampleInterval.
		// + input
		//   - delta_t: read the image sequence every delta_t time.
		void setSampleInterval(double delta_t) {sample_interval=delta_t;}

		// setNormalEstimation
		// - set the integral image normal estimation method;
		// - and some necessary parameters;
		void setNormalEstimation(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA,
								 pcl::Normal>::NormalEstimationMethod method, 
								 float MaxDepthChangeFactor, float NormalSmoothingSize);

		// getScan.
		// - return the pointer to the scan loaded from depth and rgb image;
		// - load
		//   - img_rgb,img_depth;
		//   - point_cloud, normal_cloud, pixel_cloud;
		Scan* getScan() {return scan;}

		// loadScan
		// - load the rgb and depth images to point_cloud, normal_cloud and pixel_cloud;
		void loadScan();

		// Initialize
		// + input
		//   - time_start: read the image sequence from time_start.
		// + function
		//   - open rgb.txt and depth.txt;
		//   - set timestamp to the start time;
		void Initialize(double time_start = 0);

	private:
		// SampleDataset
		// - sample the image sequence at next time step.
		void SampleDataset();

		// scan
		// - img_rgb,img_depth;
		// - point_cloud, normal_cloud, pixel_cloud;
		Scan *scan;

		// integral image normal estimation method;
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimate_integral;

		// camera intrinsic parameters;
		double fx,fy,cx,cy,factor,dx,dy;
		// width and height of the image;
		int Width,Height;
		// the time interval that sample the sequence;
		double sample_interval;
		// path of the root folder that locate the image sequence;
		const std::string freiburg;
		// file stream of rgb.txt and depth.txt;
		std::ifstream fp_rgb,fp_depth;
		// timestamp controlling the sample of the sequence;
		double timestamp_rgb, timestamp_depth, timestamp;
		// filename_rgb - rgb/*.png
		// filename_depth - depth/*.png
		std::string filename_rgb, filename_depth;

		bool debug;
	};
}
