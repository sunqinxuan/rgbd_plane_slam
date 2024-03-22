/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-30 10:38
#
# Filename:		main.cpp
#
# Description: test on the plane based slam.
#
===============================================*/
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/features/integral_image_normal.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/make_shared.hpp>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/Eigen/StdVector>
#include "DataReading.h"
#include "PlaneExtraction_RANSAC.h"
#include "PlaneMatching_Interptree.h"
#include "PlaneFilter.h"
#include "CameraPoseEstimation_PlaneParas.h"
#include "CameraPoseEstimation_ShadowSM.h"
#include "MapIncremental.h"

void plane2cloud(Plane *plane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_show)
{
	pcl::PointXYZRGBA point_tmp;
	for(std::vector<Point>::iterator it=plane->points_in.begin();it!=plane->points_in.end();++it)
	{
		point_tmp.x=it->xyz(0);	
		point_tmp.y=it->xyz(1);	
		point_tmp.z=it->xyz(2);	
		point_tmp.r=(unsigned char)(it->rgb(0)*255);
		point_tmp.g=(unsigned char)(it->rgb(1)*255);
		point_tmp.b=(unsigned char)(it->rgb(2)*255);
		//std::cout<<point_tmp.x<<","<<point_tmp.y<<","<<point_tmp.z<<","<<point_tmp.r<<","<<point_tmp.g<<","<<point_tmp.b<<std::endl;
		cloud_show->push_back(point_tmp);
	}
	//for(int j=0;j<plane->num_points;j++)
	//{
	//	point_tmp.x=plane->points_in[j].xyz(0);	
	//	point_tmp.y=plane->points_in[j].xyz(1);	
	//	point_tmp.z=plane->points_in[j].xyz(2);	
	//	point_tmp.r=(unsigned char)(plane->points_in[j].rgb(0)*255);
	//	point_tmp.g=(unsigned char)(plane->points_in[j].rgb(1)*255);
	//	point_tmp.b=(unsigned char)(plane->points_in[j].rgb(2)*255);
	//	//std::cout<<point_tmp.x<<","<<point_tmp.y<<","<<point_tmp.z<<","<<point_tmp.r<<","<<point_tmp.g<<","<<point_tmp.b<<std::endl;
	//	cloud_show->push_back(point_tmp);
	//}
}

int main (int argc, char *argv[])
{
	// some default settings;
	std::string sequence_name="/home/sun/dataset/rgbd_dataset_freiburg1_xyz";
	int bins_d=1,bins_theta=10,bins_phy=20;
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::NormalEstimationMethod method=pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::COVARIANCE_MATRIX;
	double MaxDepthChangeFactor=0.02f, NormalSmoothingSize=10.0f;
	double time_start=1305031104.775327;
	double time_interval=0.5;
	bool debug=true;

	for(int i=1;i<argc;i++)
	{
		if(strcmp(argv[i],"-h")==0)
		{
			std::cout<<"-debug\tif debugging;"<<std::endl
					 <<"-ds\tdataset sequence name = /home/sun/dataset/rgbd_dataset_freiburg1_xyz;"<<std::endl
					 <<"-gs\tgrid size (bins_theta=10, bins_phy=20, bins_d=1);"<<std::endl
					 <<"-ne\tintegral image normal estimation method (method=1, max_depth_change=0.02, smoothing_size=10.0)  (method - 1:COVARIANCE_MATRIX, 2:AVERAGE_3D_GRADIENT, 3:AVERAGE_DEPTH_CHANGE)"<<std::endl
					 <<"-st\tstart time = 0;"<<std::endl
					 <<"-ti\ttime interval = 0.5;"<<std::endl;
			return 0;
		}
		if(strcmp(argv[i],"-debug")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				debug=true;
			if(strcmp(argv[i+1],"0")==0)
				debug=false;
		}
		if(strcmp(argv[i],"-ds")==0)
		{
			sequence_name=argv[i+1];
		}
		if(strcmp(argv[i],"-gs")==0)
		{
			bins_theta=atoi(argv[i+1]);
			bins_phy=atoi(argv[i+2]);
			bins_d=atoi(argv[i+3]);
		}
		if(strcmp(argv[i],"-ne")==0)
		{
			if(strcmp(argv[i+1],"COVARIANCE_MATRIX"))
			{
				method=pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::COVARIANCE_MATRIX;
			}
			else if(strcmp(argv[i+1],"AVERAGE_3D_GRADIENT"))
			{
				method=pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::AVERAGE_3D_GRADIENT;
			}
			else if(strcmp(argv[i+1],"AVERAGE_DEPTH_CHANGE"))
			{
				method=pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::AVERAGE_DEPTH_CHANGE;
			}
			MaxDepthChangeFactor=atof(argv[i+2]);
			NormalSmoothingSize=atof(argv[i+3]);
		}
		if(strcmp(argv[i],"-st")==0)
		{
			time_start=atof(argv[i+1]);
		}
		if(strcmp(argv[i],"-ti")==0)
		{
			time_interval=atof(argv[i+1]);
		}
	}

	pcl::visualization::CloudViewer viewer("Viewer");
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud;// (new pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
	//pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;// (new pcl::PointCloud<pcl::Normal>::Ptr);
	//pcl::PointCloud<pcl::PointXY>::Ptr pixel_cloud;// (new pcl::PointCloud<pcl::PointXY>::Ptr);
	Plane *tmp_ptr_plane=new Plane;
	std::vector<Plane*> plane_map, plane_cur, plane_ref;
	std::vector<PlanePair> matched_planes;
	CameraPose camera_pose;
	Eigen::Matrix4f transform=Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_scan_global(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// definitions for debugging;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZRGBA>);

	DataReading *data_reading=new sun::DataReading(sequence_name);
	data_reading->setDebug(debug);
	data_reading->Initialize(time_start);
	data_reading->setSampleInterval(time_interval);
	data_reading->setNormalEstimation(method, MaxDepthChangeFactor, NormalSmoothingSize);

	PlaneExtraction_RANSAC *extract=new PlaneExtraction_RANSAC;
	extract->setDebug(debug);
	sun::PlaneMatching_Interptree *matching =new PlaneMatching_Interptree;
	matching->setDebug(debug);

	PlaneFilter *plane_filter=new PlaneFilter;
	plane_filter->setDebug(debug);

	CameraPoseEstimation_PlaneParas *pose_estimation=new CameraPoseEstimation_PlaneParas;
	pose_estimation->setDebug(debug);

	CameraPoseEstimation_ShadowSM *pose_estimation_sm=new CameraPoseEstimation_ShadowSM;

	MapIncremental *map=new MapIncremental;

	sun::Scan *scan_ref;
	sun::Scan *scan_cur;

	timeval start, end;
	char ch;
	double timeused;
	int first=0;
	int filenum = first;
	while(filenum<10)
	{
		std::cout<<std::endl<<"frame "<<filenum<<" **********************"<<std::endl;

		// load point cloud from the image sequence;
		gettimeofday(&start,NULL);
		data_reading->loadScan();
		scan_cur=data_reading->getScan();
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		std::cout<<"time reading data from freiburg:"<<timeused<<std::endl;
		//viewer.showCloud(scan_cur->point_cloud);
		//std::cout<<"Press enter to continue...\n"<<std::endl;
		//ch=std::cin.get();

		// load the point data to PlaneExtraction module;
		gettimeofday(&start,NULL);
		extract->loadScan(scan_cur);
		if(!extract->LoadPoints())
		{
			cout<<"no points for PlaneExtraction structure"<<endl;
			return 0;
		}
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		cout<<"time loading points:"<<timeused<<"ms"<<endl;

		// extract planes;
		gettimeofday(&start,NULL);
		extract->ExtractPlanes();
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		cout<<"time extracting planes:"<<timeused<<"ms"<<endl;

		// show the extracted planes;
		cloud_show->clear();
		std::cout<<"extracted planes:"<<std::endl;
		//for(int i=0;i<extract->GetPlaneNum();i++)
		//{
		//	tmp_ptr_plane=extract->GetPlane(i);
		//	std::cout<<i<<","<<tmp_ptr_plane<<","<<tmp_ptr_plane->normal.transpose()<<std::endl;
		//	plane2cloud(tmp_ptr_plane,cloud_show);
		//}
		for(int i=0;i<scan_cur->planes.size();i++)
		{
			plane2cloud(scan_cur->planes[i],cloud_show);
		}
		viewer.showCloud(cloud_show);
		std::cout<<"Press enter to continue...\n"<<std::endl;
		ch=std::cin.get();

		// plane filter;
		plane_filter->loadScan(scan_cur);
		plane_filter->filter();
		for(int i=0;i<scan_cur->planes.size();i++)
		{
			plane_filter->filter(scan_cur->planes[i]);
		}

		if(filenum>first)
		{
			std::cout<<"planes from reference frame:"<<std::endl;
			for(int i=0;i<plane_ref.size();i++)
			{
				cout<<plane_ref[i]->index<<","<<plane_ref[i]->normal.transpose()<<","<<plane_ref[i]->d<<endl;
			}
			// match the planes from two successive frames;
			gettimeofday(&start,NULL);
			matching->loadPlanes(plane_ref,plane_cur);
			matching->Match(matched_planes);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			cout<<"time matching planes from two frames:"<<timeused<<"ms"<<endl;
			//matching->depthFirstSearch();
			//matching->breadthFirstTravel();

			cout<<"matched planes: ";
			for(int i=0;i<matched_planes.size();i++)
			{
				cout<<"<"<<matched_planes[i].cur->index<<","<<matched_planes[i].ref->index<<">,";
			}
			cout<<endl;

			gettimeofday(&start,NULL);
			pose_estimation->loadMatchedPlanes(matched_planes);
			pose_estimation->Pose2AlignPlanes();
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			cout<<"time aligning two sets of planes:"<<timeused<<"ms"<<endl;

			gettimeofday(&start,NULL);
			pose_estimation_sm->loadMatchedPlanes(matched_planes);
			pose_estimation_sm->setPointPairs();
			pose_estimation_sm->loadDepthMap(scan_cur->point_cloud);
			pose_estimation_sm->WhatCauseShadow();
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			cout<<"time of shadow SM:"<<timeused<<"ms"<<endl;
		}
		plane_ref.clear();
		for(int i=0;i<plane_cur.size();i++)
		{
			plane_ref.push_back(plane_cur[i]);	
		}
		//camera_pose=pose_estimation->getPoseAlignPlanes();
		//transform=transform*camera_pose.getTransform();
		//std::cout<<"transform:"<<std::endl<<transform<<std::endl;
		//pcl::transformPointCloud(*scan_cur->point_cloud,*current_scan_global,transform);
		//*global_map=*global_map+*current_scan_global;

		//viewer.showCloud(global_map);
		cloud_show->clear();
		Pixel tmp_pixel;
		pcl::PointXYZRGBA tmp_point;
		for(int i=0;i<scan_cur->point_cloud->size();i++)
		{
			tmp_point=scan_cur->point_cloud->at(i);
			tmp_point.r=255;
			tmp_point.g=255;
			tmp_point.b=255;
			cloud_show->push_back(tmp_point);
		}
		for(int i=0;i<plane_cur.size();i++)
		{
//			for(int j=0;j<plane_cur[i]->num_points;j++)
//			{
//				tmp_point.x=plane_cur[i]->points_in[j].xyz(0);
//				tmp_point.y=plane_cur[i]->points_in[j].xyz(1);
//				tmp_point.z=plane_cur[i]->points_in[j].xyz(2);
//				tmp_point.r=255;
//				tmp_point.g=255;
//				tmp_point.b=255;
//				cloud_show->push_back(tmp_point);
//			}
			for(int j=0;j<plane_cur[i]->ptr_points_edge.size();j++)
			{
				tmp_point.x=plane_cur[i]->ptr_points_edge[j]->xyz(0);
				tmp_point.y=plane_cur[i]->ptr_points_edge[j]->xyz(1);
				tmp_point.z=plane_cur[i]->ptr_points_edge[j]->xyz(2);
				tmp_point.r=0;
				tmp_point.g=0;
				tmp_point.b=255;
				cloud_show->push_back(tmp_point);
			}
			for(int j=0;j<plane_cur[i]->pixels_object.size();j++)
			{
				tmp_point=scan_cur->point_cloud->at(plane_cur[i]->pixels_object[j].getIndex());
				tmp_point.r=255;
				tmp_point.g=0;
				tmp_point.b=0;
				cloud_show->push_back(tmp_point);
			}
		}
//		for(int i=0;i<pose_estimation_sm->getObjectPointNum();i++)
//		{
//			tmp_pixel=pose_estimation_sm->getObjectPoint(i);
//			cloud_show->push_back(scan_cur->point_cloud->at(tmp_pixel.getIndex()));
//			//std::cout<<tmp_pixel.u<<","<<tmp_pixel.v<<":"<<scan_cur->point_cloud->at(tmp_pixel.getIndex()).x<<","<<scan_cur->point_cloud->at(tmp_pixel.getIndex()).y<<","<<scan_cur->point_cloud->at(tmp_pixel.getIndex()).z<<std::endl;
//		}
		//viewer.showCloud(cloud_show);
		cv::imshow("scan",scan_cur->img_rgb);
		cv::waitKey(0);
		viewer.showCloud(scan_cur->point_cloud);
		std::cout<<"Press enter to continue...\n"<<std::endl;
		ch=std::cin.get();
		filenum++;
	}
	return 0;
}
