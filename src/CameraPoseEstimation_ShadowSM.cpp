/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-30 09:23
#
# Filename:		CameraPoseEstimation_ShadowSM.cpp
#
# Description: 
#
===============================================*/
#include "CameraPoseEstimation_ShadowSM.h"
#include <sys/time.h>

namespace sun
{
	CameraPoseEstimation_ShadowSM::CameraPoseEstimation_ShadowSM()
	{
	}

	CameraPoseEstimation_ShadowSM::~CameraPoseEstimation_ShadowSM()
	{
	}
		
	void CameraPoseEstimation_ShadowSM::setPointPairs()
	{
		timeval start, end;
		double timeused;
		PointPair tmp_pointpair;
		int K=1;
		ANNpointArray ref_points;
		ANNpoint query_point=annAllocPt(3); // allocate the space for query point;
		ANNidxArray point_index;
		ANNdistArray distance;

		for(int i=0;i<matched_planes.size();i++) // for each plane pair;
		{
			ref_points=annAllocPts(matched_planes[i].ref->num_points,3);
			point_index=new ANNidx[K];
			distance=new ANNdist[K];
			for(int j=0;j<matched_planes[i].ref->num_points;j++)
			{
				ref_points[j][0]=matched_planes[i].ref->points_in[j].xyz(0);
				ref_points[j][1]=matched_planes[i].ref->points_in[j].xyz(1);
				ref_points[j][2]=matched_planes[i].ref->points_in[j].xyz(2);
			}
			kdtree=new ANNkd_tree(ref_points,matched_planes[i].ref->num_points,3);

			gettimeofday(&start,NULL);
			for(int j=0;j<matched_planes[i].cur->num_points;j++)
			{
				query_point[0]=matched_planes[i].cur->points_in[j].xyz(0);
				query_point[1]=matched_planes[i].cur->points_in[j].xyz(1);
				query_point[2]=matched_planes[i].cur->points_in[j].xyz(2);
				kdtree->annkSearch(query_point,K,point_index,distance,0);
				for(int k=0;k<K;k++)
				{
					tmp_pointpair.cur=&(matched_planes[i].cur->points_in[j]);
					tmp_pointpair.ref=&(matched_planes[i].ref->points_in[point_index[k]]);
				}
			}
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time of kdtree search:"<<timeused<<"ms"<<std::endl;
			annDeallocPts(ref_points);
			delete point_index;
			delete distance;
			delete kdtree;

			std::cout<<i<<","<<matched_planes[i].ref->points_in.size()<<","<<matched_planes[i].cur->points_in.size()<<std::endl;
		}
		std::cout<<"point pairs:"<<matched_points.size()<<std::endl;
	}

	void CameraPoseEstimation_ShadowSM::loadDepthMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud)
	{
		for(int i=0;i<point_cloud->width*point_cloud->height;i++)
		{
			depth_map.push_back(point_cloud->at(i).z);
		}
	}

	void CameraPoseEstimation_ShadowSM::WhatCauseShadow()
	{
		std::ofstream fp;
		fp.open("object.txt",std::ios::out);
		Pixel tmp_pixel,tmp_pixel_nb;
		int tmp_index,u,v;
		bool flag_found_obj=false;
		for(int i=0;i<matched_planes.size();i++) // for each plane in current frame;
		{
			for(int j=0;j<matched_planes[i].cur->ptr_points_edge.size();j++) // for each edge point;
			{
				tmp_pixel.u=matched_planes[i].cur->ptr_points_edge[j]->u;
				tmp_pixel.v=matched_planes[i].cur->ptr_points_edge[j]->v;
				if(tmp_pixel.u<10||tmp_pixel.v<10||tmp_pixel.u>469||tmp_pixel.v>629)
					continue;
				for(int r=1;r<10;r++) // a certain radius;
				{
//					u=tmp_pixel.u-r;
//					for(v=tmp_pixel.v-r+1;v<=tmp_pixel.v+r-1;v++)
//					{
//						if(flag_found_obj)
//							continue;
//						tmp_pixel_nb=Pixel(u,v);
//						if(depth_map[tmp_pixel.getIndex()]-depth_map[tmp_pixel_nb.getIndex()]>0.1)
//						{
//							object_points.push_back(Pixel(u,v));
//							flag_found_obj=true;
//						}
//					}
//					u=tmp_pixel.u+r;
//					for(v=tmp_pixel.v-r+1;v<=tmp_pixel.v+r-1;v++)
//					{
//						if(flag_found_obj)
//							continue;
//						tmp_pixel_nb=Pixel(u,v);
//						if(depth_map[tmp_pixel.getIndex()]-depth_map[tmp_pixel_nb.getIndex()]>0.1)
//						{
//							object_points.push_back(Pixel(u,v));
//							flag_found_obj=true;
//						}
//					}
//					v=tmp_pixel.v-r;
//					for(u=tmp_pixel.u-r+1;u<=tmp_pixel.u+r-1;v++)
//					{
//						if(flag_found_obj)
//							continue;
//						tmp_pixel_nb=Pixel(u,v);
//						if(depth_map[tmp_pixel.getIndex()]-depth_map[tmp_pixel_nb.getIndex()]>0.1)
//						{
//							object_points.push_back(Pixel(u,v));
//							flag_found_obj=true;
//						}
//					}
//					v=tmp_pixel.v+r;
//					for(u=tmp_pixel.u-r+1;u<=tmp_pixel.u+r-1;v++)
//					{
//						if(flag_found_obj)
//							continue;
//						tmp_pixel_nb=Pixel(u,v);
//						if(depth_map[tmp_pixel.getIndex()]-depth_map[tmp_pixel_nb.getIndex()]>0.1)
//						{
//							object_points.push_back(Pixel(u,v));
//							flag_found_obj=true;
//						}
//					}
					for(int u=tmp_pixel.u-5;u<tmp_pixel.u+5;u++)
					{
						for(int v=tmp_pixel.v-5;v<tmp_pixel.v+5;v++)
						{
							tmp_pixel_nb=Pixel(u,v);
							tmp_index=tmp_pixel_nb.getIndex();
							if(depth_map[tmp_pixel.getIndex()]-depth_map[tmp_pixel_nb.getIndex()]>0.1)
							//if(depth_map[tmp_index]>-1)//&&abs(depth_map[tmp_index]-matched_planes[i].cur->ptr_points_edge[j]->xyz(2))>0.1)
							{
								//object_points.push_back(tmp_pixel_nb);
								matched_planes[i].cur->pixels_object.push_back(tmp_pixel_nb);
								//depth_map[tmp_index]=-2;
							}
						}
					}
				}
			}
		}
		// debug
		//cv::Mat depth_img=cv::Mat::zeros(480,640,CV_8UC1);
		//for(int u=0;u<480;u++)
		//{
		//	for(int v=0;v<640;v++)
		//	{
		//		tmp_pixel.u=u;
		//		tmp_pixel.v=v;
		//		depth_img.at<unsigned char>(tmp_pixel.u,tmp_pixel.v)=int(depth_map[tmp_pixel.getIndex()]*50);
		//	}
		//}
		//cv::imshow("depth loaded", depth_img);
		//cv::waitKey(0);
		// debug done
		fp.close();
	}

}
