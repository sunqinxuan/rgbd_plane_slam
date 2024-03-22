/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-07-17 16:17
#
# Filename:		Basics.h
#
# Description: some basic structures.
#
===============================================*/
#pragma once
#include <stdio.h>
#include <fstream>
#include <vector>
#include <math.h>
#include <list>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <opencv2/core/core.hpp>

namespace sun
{
	struct Pixel
	{
		// pixel coordinate;
		int u,v;
		// number of pixels in a row;
		int step;

		Pixel() {u=0;v=0;step=640;}
		Pixel(int i,int j) {u=i;v=j;step=640;}
		Pixel(int i,int j,int s) {u=i;v=j;step=s;}

		int getIndex() {return u*step+v;}

		// to use it as the index in std::map structure;
		bool operator < (const Pixel &x) const
		{
			if(this->u*this->step+this->v<x.u*x.step+x.v)
				return true;
			else
				return false;
		}
	};

	// index in the pps (theta-phy-d)
	//struct Pixel_pps
	//{
	//	int theta,phy,d,layer;
	//	int step_theta,step_phy;
	//	bool operator < (const Pixel_pps &x) const
	//	{
	//		if(this->layer==x.layer)
	//		{
	//			if(this->d*this->step_theta*this->step_phy+this->theta*this->step_phy+this->phy < x.d*x.step_theta*x.step_phy+x.theta*x.step_phy+x.phy)
	//				return true;
	//			else
	//				return false;
	//		}
	//		else if(this->layer<x.layer)
	//			return true;
	//		else
	//			return false;
	//	}
	//};

	struct Point
	{
		// pps: coordinate in the PPS for the local plane parameters (after Rotation2eigen);
		// rgb: [0,1] RGB information of the point;
		// xyz: coordinate in the camera coordinate system;
		// normal: local plane normal in original camera coordinate system;
		Eigen::Vector3d pps, rgb, xyz, normal;

		// u,v: pixel coordinate;
		int u,v; // u<480, v<640

		// cov: covariance for icp;
		Eigen::Matrix3d cov; 
		double weight;
	};

	struct Cell
	{
		// number of points in the cell;
		int num_points;

		// normal,d: plane coordinate in the camera coordinate system;
		Eigen::Vector3d normal;
		double d;

		// avg_pps,cov_pps: computed from the PPS coordinates of the inside points;
		// avg_rgb,cov_rgb: computed from the RGB of the inside points;
		Eigen::Vector3d avg_pps, avg_rgb;
		Eigen::Matrix3d cov_pps, cov_rgb;
		
		bool isEmpty; // whether the cell is empty;
		bool isBottom; // whether the cell is on bottom level;

		// points in the cell;
		std::vector<Point> points_in;

		// indices w.r.t. point_cloud and normal_cloud;
		// to index the cell points in the point_cloud;
		pcl::PointIndices::Ptr inliers;

		// the followings are used in the sting
		// temprary existence
		// deleted when PlaneExtraction_STING.cpp is modified
		//int layer;
		//bool relevant;
		//Cell *father;
		//Cell *child[8];
	};

	struct Plane
	{
		// plane parameters;
		Eigen::Vector3d normal;
		double d;
		
		// avg_rgb, cov_rgb: computed from the points_in
		Eigen::Vector3d avg_rgb;
		Eigen::Matrix3d cov_rgb;

		// all the points on plane;
		std::vector<Point> points_in;
		// number of points on plane;
		int num_points;

		// for debugging
		int index;
		
		// delete *******************************************************
		//
		// avg_pps, avg_rgb, cov_pps, cov_rgb: computed from the points_in
		// avg_pps, cov_pps: after Rotation2eigen
		Eigen::Vector3d avg_pps;
		Eigen::Matrix3d cov_pps;

		// not used temporarily;
		Eigen::Vector3d pps, rgb;

		// pointer to the points on the shadow edge;
		std::vector<Point*> ptr_points_edge;

		// object points corresponding to the edge points;
		std::vector<Point> points_object;
		std::vector<Pixel> pixels_object;

	};
	
	struct PointPair
	{
		Point *ref;
		Point *cur;
	};

	struct CameraPose
	{
		Eigen::Vector3d position;
		Eigen::Matrix3d Rotation;
		// eular angle: Z-Y-X
		Eigen::Vector3d Eulars() {return Rotation.eulerAngles(2,1,0);}
		Eigen::Quaterniond Quat() {return Eigen::Quaterniond(Rotation);}
		Eigen::Matrix4f getTransform()
		{
			Eigen::Matrix4f transform;
			transform.setIdentity();
			for(int i=0;i<3;i++)
			{
				transform(i,3)=(float)position(i);
				for(int j=0;j<3;j++)
				{
					transform(i,j)=(float)Rotation(i,j);
				}
			}
			return transform;
		}
	};

	struct Scan
	{
		CameraPose pose;
		std::vector<Plane*> planes;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud;
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;
		pcl::PointCloud<pcl::PointXY>::Ptr pixel_cloud;
		cv::Mat img_rgb, img_depth;
	};
}
