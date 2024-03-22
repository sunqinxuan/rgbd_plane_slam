/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-07-17 16:41
#
# Filename:		PlaneExtraction.h
#
# Description: base class for all the plane extraction methods.
#
===============================================*/
#pragma once
#include <stdio.h>
#include <fstream>
#include <vector>
#include <stack>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/LU>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include "Basics.h"

using namespace sun;

namespace sun
{
	// Sorted_Cell
	// - store the index of the cells;
	// - used to sort the cells in Cells_bottom;
	struct Sorted_Cell
	{
		Sorted_Cell(){}
		bool operator < (const Sorted_Cell &m)const
		{
			return num_point < m.num_point;
		}
		int index;
		int num_point;
	};

	// Cells_bottom
	// - construct the bottom grid from the original data points;
	// - then the upper levels could be constructed based on this;
	// - or it can be directly used as a orientation histogram;
	class Cells_bottom
	{
	public:

		Cells_bottom() 
		{
			bins_theta=10;bins_phy=20;bins_d=1;
			Initialize();
		}

		Cells_bottom(int theta, int phy, int d) 
		{
			bins_theta=theta;bins_phy=phy;bins_d=d;
			Initialize();
		}

		// push_point
		// - push the point point_tmp into the corresponding cell;
		// - inlier is the index w.r.t. the point_cloud;
		void push_point(Point point_tmp, int inlier);

		// ComputeCellAttribute
		// - compute cell attributes for all the cells in cell_bottom;
		// - cell attributes include
		//   - num_points
		//   - avg_pps, cov_pps (after Rotation2eigen, computed from Point.pps)
		//   - avg_rgb, cov_rgb
		//   - normal, d (plane parameter, in original frame)
		void ComputeCellAttribute(Eigen::Matrix3d Rotation2eigen=Eigen::Matrix3d::Identity());

		// SortCells
		// - sort the cells according to the number of inside points;
		// - store the sorting result in the sorted_cells;
		void SortCells();

		Cell* getCell(int i) {return &cells[i];}
		Cell* getCell(int d, int theta, int phy) {return &cells[index(d,theta,phy)];}

		int getCellSize() {return cells.size();}

		std::vector<Sorted_Cell>::iterator getHighestCell() {return sorted_cells.end()-1;}

	private:

		// Initialise
		// - malloc the cells to bins_theta*bins_phy*bins_d;
		// - all the cells are set to empty;
		void Initialize();

		int index(int d, int theta, int phy) {return d*bins_theta*bins_phy+theta*bins_phy+phy;}
		int bins_theta,bins_phy,bins_d;
		double delta_theta,delta_phy,delta_d;
		std::vector<Cell> cells;
		std::vector<Sorted_Cell> sorted_cells;
	};

	class PlaneExtraction
	{
	public:
		PlaneExtraction(): bins_theta(10), bins_phy(20), bins_d(1) {}

		PlaneExtraction(int theta, int phy, int d): bins_theta(theta), bins_phy(phy), bins_d(d) {}

		~PlaneExtraction() {}

		// some interfaces;
		void setDebug(bool d) {debug=d;}
		int GetPointsNum(void) {return points.size();}
		Point GetPoint(int i) {return points[i];}
		Plane* GetPlane(int i) {return scan->planes[i];}
		int GetPlaneNum(void) {return scan->planes.size();}
		void Clear();

		// loadScan
		// - img_rgb,img_depth;
		// - point_cloud, normal_cloud, pixel_cloud;
		void loadScan(Scan *s) {scan=s;}

		// LoadPoints
		// - compute the Rotation2eigen;
		// - push the scan points into the points and the cell_bottom;
		// - and sort the cells according to the number of points inside;
		virtual bool LoadPoints(Scan *scan);

		// ExtractPlanes
		// - extracting planes from scan data;
		// - implemented in the inheriting class;
		virtual void ExtractPlanes()=0;
		
	protected:
		bool debug;
		int bins_theta,bins_phy,bins_d;

		// scan
		// - loaded with
		//   - img_rgb,img_depth;
		//   - point_cloud, normal_cloud, pixel_cloud;
		// - returned with
		//   - planes;
		//     - planes extracted by ExtractPlanes();
		//     - implemented in the inheriting class;
		Scan *scan;

		// Rotation2eigen
		// - rotation from the original coordinate to the coordinate where 
		// - the z axis pointing to the direction with least normals;
		Eigen::Matrix3d Rotation2eigen;

		// points
		// - store the points from the current frame;
		// - normal: local plane normal;
		// - pps: coordinate in the PPS (after Rotation2eigen);
		// - xyz: coordinate in the camera frame;
		// - rgb: RGB [0,1];
//		std::vector<Point> points;

		// cells_bottom
		// - cells in the bottom level of STING;
		// - or used directly as the normal histogram to extract planes;
		Cells_bottom *cells_bottom;

		// Cartesian2PPS
		// - transform from the Cartesian space to the PPS;
		// - rotate the normal via Rotation2eigen if defined;
		Eigen::Vector3d Cartesian2PPS(Eigen::Vector3d normal, Eigen::Vector3d point, Eigen::Matrix3d Rotation2eigen=Eigen::Matrix3d::Identity());

		// computePlaneParas
		// - compute normal and d from points_in;
		// - using the least square method;
		void computePlaneParas(Plane *plane);
	};
}
