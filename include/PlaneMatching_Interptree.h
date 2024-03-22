/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-08 20:59
#
# Filename:		PlaneMatching_Interptree.h
#
# Description: 
#
===============================================*/
#pragma once
#include "PlaneMatching.h"

using namespace sun;

namespace sun
{
	//struct PlaneFeature
	//{
	//	Eigen::Vector3d normal, pps, rgb;
	//	double d;
	//	Eigen::Matrix3d cov_rgb, cov_pps;
	//};

	struct Node_InterpTree
	{
		Plane *plane_ref,*plane_cur;
		Node_InterpTree *parent;
		std::vector<Node_InterpTree*> children;
		Node_InterpTree();
		Node_InterpTree(Plane *plane_ref_, Plane *plane_cur_);
		void setParent(Node_InterpTree*);
		void insertChild(Node_InterpTree*);
		// for debugging
		int layer;
	};

	// layers: numbers of planes in referece frame
	// children per node: numbers of planes in current frame
	class InterpTree
	{
	public:
		InterpTree();
		~InterpTree();
		void Clear();
		void setDebug(bool d) {debug=d;}
		int getNodeNum() {return nodes.size();}
		Node_InterpTree* getRoot() {return root;}
		Node_InterpTree* getMaxInterp() {return leaf_max_interp;}
		bool Construct(std::vector<Plane*> &planes_ref_,std::vector<Plane*> &planes_cur_);
		
	private:
		bool debug;
		Node_InterpTree *root;
		std::vector<Node_InterpTree*> nodes;
		Node_InterpTree *leaf_max_interp;
		double thres_color; // thres for consistent_1
		double thres_delta_angle; // (consistent_2) if delta_normal_angle<thres then the planes are parallel
		double thres_delta_d; // (consistent_2) if delta_d<thres then the plane pairs are coincident
		bool consistent_1(Node_InterpTree *node);
		bool consistent_2(Node_InterpTree *node1, Node_InterpTree *node2);
		bool consistent_3(Node_InterpTree *node1, Node_InterpTree *node2, Node_InterpTree *node3);
		bool isLeaf(Node_InterpTree *node);
	};

	class PlaneMatching_Interptree : public PlaneMatching
	{
	public:
		PlaneMatching_Interptree();
		~PlaneMatching_Interptree();
		//void setDebug(bool d) {debug=d; interp_tree->setDebug(d);}
		virtual void Match(std::vector<PlanePair> &matched_planes);
		//void depthFirstSearch();
		//void breadthFirstTravel();
	private:
		//bool debug;
		InterpTree *interp_tree;
	};
}
