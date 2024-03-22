/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-14 20:35
#
# Filename:		MapIncremental.cpp
#
# Description: 
#
===============================================*/
#include "MapIncremental.h"

namespace sun
{
	MapIncremental::MapIncremental()
	{}

	MapIncremental::~MapIncremental()
	{}

	void MapIncremental::addScan(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud, CameraPose Rt)
	{
		Scan scan_tmp;
		scan_tmp.point_cloud=point_cloud;
		scans.push_back(scan_tmp);
	}

	void MapIncremental::showScans()
	{
		//pcl::visualization::CloudViewer viewer("show map");
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr show_map(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_scan(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//Eigen::Matrix4f transform;
		//for(int i=0;i<camera_traj.size();i++)
		//{
		//	transform=transform*camera_traj[i].getTransform();
		//	pcl::transformPointCloud(*(scan_map[i]),*tmp_scan,transform);
		//	*show_map=*show_map+*tmp_scan;
		//}
		//viewer.showCloud(show_map);
	}
	
	//Eigen::Matrix4f MapIncremental::getTransform(CameraPose *pose)
	//{
	//	Eigen::Matrix4f transform;
	//	transform.setIdentity();
	//	for(int i=0;i<3;i++)
	//	{
	//		transform(i,3)=(float)pose.position(i);
	//		for(int j=0;j<3;j++)
	//		{
	//			transform(i,j)=(float)pose.Rotation(i,j);
	//		}
	//	}
	//	return transform;
	//}
}
