/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-14 21:06
#
# Filename:		MapIncremental.h
#
# Description: 
#
===============================================*/
#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include "Basics.h"
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>

namespace sun
{
	class MapIncremental
	{
	public:
		MapIncremental();
		~MapIncremental();
		//void addScan(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud, CameraPose Rt, std::vector<Plane*> planes_cur, std::vector<PlanePair> matched_planes);
		void addScan(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud, CameraPose Rt);
		void showScans();
	private:
		std::vector<Scan> scans;
		//std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> scan_map;
		//std::vector<Plane*> plane_map;
		//std::vector<CameraPose> camera_traj;
		//Eigen::Matrix4f getTransform(CameraPose pose);
	};
}
