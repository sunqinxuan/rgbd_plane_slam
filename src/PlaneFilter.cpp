/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-04-10 14:53
#
# Filename:		PlaneFilter.cpp
#
# Description: 
#
===============================================*/
#include "PlaneFilter.h"

namespace sun
{
	PlaneFilter::PlaneFilter()
	{
		cx=318.6;//331;//324.7;//308.8;//319.5;
		cy=255.3;//234;//250.1;//253;//239.5;
		// temp_color (temp_edge)
		// 	| 1  1 1 |
		// 	| 1 -8 1 |
		// 	| 1  1 1 |
		temp_color.setZero();
		temp_color(0,1)=1;
		temp_color(1,0)=1;
		temp_color(1,2)=1;
		temp_color(2,1)=1;
		temp_color(0,0)=1;
		temp_color(0,2)=1;
		temp_color(2,0)=1;
		temp_color(2,2)=1;
		temp_color(1,1)=-8;
		temp_edge=temp_color;
		temp_density=temp_color;
		// temp_density
		// 	| 1 1 1 |
		// 	| 1 0 1 |
		// 	| 1 1 1 |
		temp_density(1,1)=0;
	}

	void PlaneFilter::filter()
	{
		cv::Mat tmp_gaussian_rgb,tmp_canny_rgb;
		cv::Mat tmp_gaussian_depth,tmp_canny_depth;

		cv::GaussianBlur(scan->img_rgb, tmp_gaussian_rgb, cv::Size(3,3),0);
		cv::cvtColor(tmp_gaussian_rgb,tmp_gaussian_rgb,cv::COLOR_BGR2GRAY);
		cv::Canny(tmp_gaussian_rgb,tmp_canny_rgb,100,300);
		std::cout<<"img_rgb:"<<scan->img_rgb.depth()<<","<<scan->img_rgb.channels()<<std::endl;
		cv::imshow("Canny",tmp_canny_rgb);
		cv::waitKey(0);

		cv::Mat tmp_depth(480,640,CV_8UC1);
		for(int i=0;i<480;i++)
		{
			for(int j=0;j<640;j++)
			{
				tmp_depth.at<unsigned char>(i,j)=int(scan->img_depth.at<unsigned int>(i,j)*255/65535);
			}
		}

		std::cout<<"img_depth:"<<tmp_depth.depth()<<","<<tmp_depth.channels()<<","<<scan->img_depth.size()<<","<<tmp_depth.size()<<std::endl;
		cv::GaussianBlur(tmp_depth, tmp_gaussian_depth, cv::Size(3,3),0);
		//cv::cvtColor(tmp_gaussian_depth,tmp_gaussian_depth,cv::COLOR_BGR2GRAY);
		cv::Canny(tmp_gaussian_depth,tmp_canny_depth,100,300);
		cv::imshow("Canny",tmp_canny_depth);
		cv::waitKey(0);
	}

	void PlaneFilter::filter(Plane *plane)
	{
		loadPlane(plane);
		color2grey();

		std::ofstream fp;
		fp.open("response.txt",std::ios::out);
		// the two mats are just for display;
		cv::Mat color_img=cv::Mat::zeros(480,640,CV_8UC1);
		cv::Mat edge_img=cv::Mat::zeros(480,640,CV_8UC1);
		cv::Mat plane_img=cv::Mat::zeros(480,640,CV_8UC3);
		double color_response_tmp,edge_response_tmp;
		plane->ptr_points_edge.clear();
		Eigen::Matrix3d R;
		R=rotation_plane2camera(plane->normal);
		for(std::map<Pixel,Point*>::iterator it=index_map.begin();it!=index_map.end();it++)
		{
			color_response_tmp=DepthWeight(it->first)*ColorResponse(it->first,temp_color);
			edge_response_tmp=DepthWeight(it->first)*EdgeResponse(it->first,temp_edge);//CenterWeight(it->first)*
			color_response.insert(std::map<Pixel,double>::value_type(it->first,color_response_tmp));
			edge_response.insert(std::map<Pixel,double>::value_type(it->first,edge_response_tmp));
			// just for display;
			color_img.at<unsigned char>(it->first.u,it->first.v)=int(color_response_tmp*200);//int(grey.find(it->first)->second*255);//
			edge_img.at<unsigned char>(it->first.u,it->first.v)=int(edge_response_tmp*40);//int(grey.find(it->first)->second*255);
			plane_img.at<cv::Vec3b>(it->first.u,it->first.v)[0]=(unsigned char)(it->second->rgb[2]*255);
			plane_img.at<cv::Vec3b>(it->first.u,it->first.v)[1]=(unsigned char)(it->second->rgb[1]*255);
			plane_img.at<cv::Vec3b>(it->first.u,it->first.v)[2]=(unsigned char)(it->second->rgb[0]*255);
			if(debug)
				fp<<it->first.u<<","<<it->first.v<<","<<color_response_tmp<<","<<edge_response_tmp<<std::endl;
			it->second->weight=edge_response_tmp;
			it->second->cov.setIdentity();
			it->second->cov(0,0)=0.001;
			if(edge_response_tmp>1)
			{
				plane->ptr_points_edge.push_back(it->second);
				it->second->cov(1,1)=0.001;
				it->second->cov(2,2)=0.001;
			}
			it->second->cov=R*it->second->cov*R.transpose();
		}
		if(debug)
		{
			cv::imshow("plane image",plane_img);
			cv::waitKey(0);
			cv::imshow("edge filter",edge_img);
			cv::waitKey(0);
			cv::imshow("color filter",color_img);
			cv::waitKey(0);
		}
		fp.close();
	}

	void PlaneFilter::EdgeFilter()
	{
		if(debug)
		{
			std::cout<<"template:"<<std::endl;
			std::cout<<temp_edge<<std::endl;
		}

		std::ofstream fp;
		fp.open("edge_response.txt",std::ios::out);
		// the two mats are just for display;
		cv::Mat edge_img=cv::Mat::zeros(480,640,CV_8UC1);
		double edge_response_tmp;
		for(std::map<Pixel,Point*>::iterator it=index_map.begin();it!=index_map.end();it++)
		{
			edge_response_tmp=DepthWeight(it->first)*EdgeResponse(it->first,temp_edge);//CenterWeight(it->first)*
			edge_response.insert(std::map<Pixel,double>::value_type(it->first,edge_response_tmp));
			edge_img.at<unsigned char>(it->first.u,it->first.v)=int(edge_response_tmp*40);//int(grey.find(it->first)->second*255);
			if(debug)
				fp<<it->first.u<<","<<it->first.v<<","<<edge_response_tmp<<std::endl;
		}
		if(debug)
		{
			cv::imshow("edge filter",edge_img);
			cv::waitKey(0);
		}
		fp.close();
	}

	void PlaneFilter::ColorFilter()
	{
		color2grey();
		if(debug)
		{
			std::cout<<"template:"<<std::endl;
			std::cout<<temp_color<<std::endl;
		}

		std::ofstream fp;
		fp.open("color_response.txt",std::ios::out);
		// the two mats are just for display;
		cv::Mat color_img=cv::Mat::zeros(480,640,CV_8UC1);
		cv::Mat plane_img=cv::Mat::zeros(480,640,CV_8UC3);
		double color_response_tmp;
		for(std::map<Pixel,Point*>::iterator it=index_map.begin();it!=index_map.end();it++)
		{
			color_response_tmp=DepthWeight(it->first)*ColorResponse(it->first,temp_color);
			color_response.insert(std::map<Pixel,double>::value_type(it->first,color_response_tmp));
			color_img.at<unsigned char>(it->first.u,it->first.v)=int(color_response_tmp*200);//int(grey.find(it->first)->second*255);//
			plane_img.at<cv::Vec3b>(it->first.u,it->first.v)[0]=(unsigned char)(it->second->rgb[2]*255);
			plane_img.at<cv::Vec3b>(it->first.u,it->first.v)[1]=(unsigned char)(it->second->rgb[1]*255);
			plane_img.at<cv::Vec3b>(it->first.u,it->first.v)[2]=(unsigned char)(it->second->rgb[0]*255);
			if(debug)
				fp<<it->first.u<<","<<it->first.v<<","<<color_response_tmp<<std::endl;
		}
		if(debug)
		{
			cv::imshow("plane image",plane_img);
			cv::waitKey(0);
			cv::imshow("color filter",color_img);
			cv::waitKey(0);
		}
		fp.close();
	}

	void PlaneFilter::DensityFilter()
	{
		double density_tmp;
		for(std::map<Pixel,Point*>::iterator it=index_map.begin();it!=index_map.end();it++)
		{
			density_tmp=DensityResponse(it->first,temp_edge);//CenterWeight(it->first)*
			density_response.insert(std::map<Pixel,double>::value_type(it->first,density_tmp));
			//edge_img.at<unsigned char>(it->first.u,it->first.v)=int(edge_response_tmp*40);//int(grey.find(it->first)->second*255);
		}
	}

	void PlaneFilter::loadPlane(Plane *plane)
	{
		Pixel tmp_pixel;
		Point *tmp_ptr_point;
		index_map.clear();
		color_response.clear();
		edge_response.clear();
		grey.clear();
		for(std::vector<Point>::iterator it=plane->points_in.begin();it!=plane->points_in.end();++it)
		{
			tmp_pixel.u=it->u;
			tmp_pixel.v=it->v;
			tmp_ptr_point=&(*it);
			index_map.insert(std::map<Pixel,Point*>::value_type(tmp_pixel,tmp_ptr_point));
		}
	}

	Eigen::Matrix3d PlaneFilter::rotation_plane2camera(Eigen::Vector3d n)
	{
		Eigen::Matrix3d R;
		R.block<3,1>(0,0)=n;
		R(0,0)=1;
		R(0,1)=1;
		R(0,2)=-(n(0)+n(1))/n(2);
		R.block<3,1>(0,1).normalize();
		R.block<3,1>(0,2)=n.cross(R.block<3,1>(0,1));
		R.block<3,1>(0,2).normalize();
		return R;
	}

	// density response for one pixel
	double PlaneFilter::DensityResponse(Pixel index, Eigen::MatrixXd temp_density)
	{
		double response=0;
		int step=(temp_density.rows()+1)/2;
		Pixel tmp_index;
		tmp_index.step=640;
		int count=0;
		for(int i=0;i<temp_density.rows();i++)
		{
			for(int j=0;j<temp_density.cols();j++)
			{
				tmp_index.u=index.u+step*(i-1);
				tmp_index.v=index.v+step*(j-1);
				if(grey.find(tmp_index)==grey.end())
				{
					continue;
					//response+=temp_edge(i,j)*index_map.find(index)->second->xyz(2);
				}
				else
				{
					count++;
					response+=temp_density(i,j)*index_map.find(tmp_index)->second->xyz(2)-index_map.find(tmp_index)->second->xyz(2);
				}
			}
		}
		if(response<0)
			response=-response;
		return response;
	}

	// color response for one pixel
	double PlaneFilter::ColorResponse(Pixel index, Eigen::MatrixXd temp_color)
	{
		double response=0;
		int step=(temp_color.rows()+1)/2;
		Pixel tmp_index;
		tmp_index.step=640;
		for(int i=0;i<temp_color.rows();i++)
		{
			for(int j=0;j<temp_color.cols();j++)
			{
				tmp_index.u=index.u+step*(i-1);
				tmp_index.v=index.v+step*(j-1);
				if(grey.find(tmp_index)==grey.end())
				{
					response+=temp_color(i,j)*grey.find(index)->second;
				}
				else
				{
					response+=temp_color(i,j)*grey.find(tmp_index)->second;
				}
			}
		}
		if(response<0)
			response=-response;
		return response;
	}

	// edge response for one pixel
	double PlaneFilter::EdgeResponse(Pixel index, Eigen::MatrixXd temp_edge)
	{
		double response=0;
		int step=(temp_edge.rows()+1)/2;
		Pixel tmp_index;
		tmp_index.step=640;
		for(int i=0;i<temp_edge.rows();i++)
		{
			for(int j=0;j<temp_edge.cols();j++)
			{
				tmp_index.u=index.u+step*(i-1);
				tmp_index.v=index.v+step*(j-1);
				if(grey.find(tmp_index)==grey.end())
				{
					continue;
					//response+=temp_edge(i,j)*index_map.find(index)->second->xyz(2);
				}
				else
				{
					response+=temp_edge(i,j)*index_map.find(tmp_index)->second->xyz(2);
				}
			}
		}
		if(response<0)
			response=-response;
		return response;
	}

	// weight for the whole filter;
	// defined by the uncertainty of the kinect depth value;
	double PlaneFilter::DepthWeight(Pixel index)
	{
		if(index_map.find(index)->second->xyz(2)>6)
			return 0;
		else
			return 19*(5.13e-2-0.5*2.85e-3*index_map.find(index)->second->xyz(2)*index_map.find(index)->second->xyz(2));
	}

	double PlaneFilter::CenterWeight(Pixel index)
	{
		double u=index.u-cy;
		double v=index.v-cx;
		return -u*u/57600-v*v/102400+1;
	}

	// save grey information in map if needed
	void PlaneFilter::color2grey()
	{
		for(std::map<Pixel,Point*>::iterator it=index_map.begin();it!=index_map.end();it++)
		{
			// Gray = R*0.299 + G*0.587 + B*0.114
			double g=it->second->rgb(0)*0.299+it->second->rgb(1)*0.587+it->second->rgb(2)*0.114;
			grey.insert(std::map<Pixel,double>::value_type(it->first,g));
		}
	}
}
