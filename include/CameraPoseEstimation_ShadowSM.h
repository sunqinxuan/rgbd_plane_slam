/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-28 16:33
#
# Filename:		CameraPoseEstimation_ShadowSM.h
#
# Description: 
#
===============================================*/
#pragma once
#include "ANN/ANN.h"
#include "CameraPoseEstimation.h"
#include <pcl-1.8/pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace sun
{
	class CameraPoseEstimation_ShadowSM : public  CameraPoseEstimation
	{
	public:
		CameraPoseEstimation_ShadowSM();
		~CameraPoseEstimation_ShadowSM();

		void setPointPairs();
		void loadDepthMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud);
		void WhatCauseShadow();
		Pixel getObjectPoint(int i) {return object_points[i];}
		int getObjectPointNum() {return object_points.size();}

	private:
		std::vector<PointPair> matched_points;
		ANNkd_tree *kdtree;
		std::vector<double> depth_map;
		std::vector<Pixel> object_points;
	};
}
