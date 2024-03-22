/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-09 09:44
#
# Filename:		PlaneMatching_Interptree.cpp
#
# Description: 
#
===============================================*/
#include "PlaneMatching_Interptree.h"

namespace sun
{
	Node_InterpTree::Node_InterpTree()
	{
	}

	Node_InterpTree::Node_InterpTree(Plane *plane_ref_, Plane *plane_cur_)
	{
		plane_ref=plane_ref_;
		plane_cur=plane_cur_;
	}

	void Node_InterpTree::setParent(Node_InterpTree *node)
	{
		parent=node;
		node->children.push_back(this);
		this->layer=node->layer+1;
	}

	void Node_InterpTree::insertChild(Node_InterpTree *node)
	{
		children.push_back(node);
		node->parent=this;
		node->layer=this->layer+1;
	}

	InterpTree::InterpTree()
	{
		root=new Node_InterpTree;
		thres_color=0.2;
		thres_delta_angle=0.2; // 0.2rad~=11.5deg
		thres_delta_d=0.1; // 10cm
	}

	InterpTree::~InterpTree()
	{
	}

	void InterpTree::Clear()
	{
		delete root;
		for(std::vector<Node_InterpTree*>::iterator it=nodes.begin();it!=nodes.end();it++)
		{
			delete *it;
		}
		std::vector<Node_InterpTree*>().swap(nodes);
	}

	// construct interpretation tree from two plane set
	// each path down to the leaf represent a hypothesis
	// *************************************************
	// consider: the circumstances returning false
	bool InterpTree::Construct(std::vector<Plane*> &planes_ref_,std::vector<Plane*> &planes_cur_)
	{
		// destruct the existing interpretation tree
		Clear();
		root=new Node_InterpTree;
		root->layer=0;
		Node_InterpTree *tmp_node, *insert_node, *node_attach, *interp_node2, *interp_node3;
		std::vector<Node_InterpTree*> nodes_tmp;
		bool flag_consistent=true;
		tmp_node=new Node_InterpTree();
		int interp_length_max=0, interp_length;
		leaf_max_interp=root;
		//std::cout<<"nodes:"<<nodes.size()<<std::endl;
		if(debug)
		{
			std::cout<<"plane_cur:"<<planes_cur_.size()<<std::endl;
			for(int i=0;i<planes_cur_.size();i++)
			{
				std::cout<<planes_cur_[i]->index<<","<<planes_cur_[i]->normal.transpose()<<std::endl;
			}
			std::cout<<"plane_ref:"<<planes_ref_.size()<<std::endl;
			for(int i=0;i<planes_ref_.size();i++)
			{
				std::cout<<planes_ref_[i]->index<<","<<planes_ref_[i]->normal.transpose()<<std::endl;
			}
		}
		// for any two planes out of planes_cur_ and planes_ref_
		for(std::vector<Plane*>::iterator it_cur=planes_cur_.begin();it_cur!=planes_cur_.end();it_cur++)
		{
			for(std::vector<Plane*>::iterator it_ref=planes_ref_.begin();it_ref!=planes_ref_.end();it_ref++)
			{
				tmp_node->plane_cur=*it_cur;
				tmp_node->plane_ref=*it_ref;
				if(debug)
				{
					std::cout<<std::endl<<"now in: <"<<tmp_node->plane_cur->index<<","<<tmp_node->plane_ref->index<<"> "<<std::endl;
				}
				if(!consistent_1(tmp_node))
					continue;
				// it has to be ensured that the plane index 0 in planes_cur_ must have a correspondence in planes_ref_
				// and there consistent_1 must be satisfied
				// otherwise there will be no node insert to the root
				//if(it_cur==planes_cur_.begin())
				{
					insert_node=new Node_InterpTree(*it_ref,*it_cur);
					root->insertChild(insert_node);
					nodes.push_back(insert_node);
					if(debug)
					{
						std::cout<<"insert <"<<insert_node->plane_cur->index<<","<<insert_node->plane_ref->index<<"> to root"<<std::endl;
					}
					//continue;
				}
				if(debug)
				{
					std::cout<<"nodes in the tree: ";
					for(std::vector<Node_InterpTree*>::iterator it=nodes.begin();it!=nodes.end();it++)
					{
						std::cout<<"<"<<(*it)->plane_cur->index<<","<<(*it)->plane_ref->index<<">,";
					}
					std::cout<<std::endl;
				}
				for(int i_node=nodes.size()-1;i_node>=0;i_node--)
				{
					flag_consistent=true;
					if(debug)
					{
						std::cout<<"node: "<<nodes[i_node]->plane_cur->index<<","<<nodes[i_node]->plane_ref->index<<std::endl;
					}
					//if((*it_node)->plane_cur->index==tmp_node->plane_cur->index-1)
					{
						node_attach=nodes[i_node];
						interp_node2=node_attach;
						
						//Interp.clear();
						//while(interp_node2!=root)
						//{
						//	if(interp_node2->plane_ref==tmp_node->plane_ref)
						//	{
						//		flag_consistent=false;
						//		break;
						//	}
						//	Interp.push_back(interp_node2);
						//	interp_node2=interp_node2->parent;
						//}
						////*******************************
						//if(debug)
						//{
						//	std::cout<<"Interp:";
						//	for(std::vector<Node_InterpTree*>::iterator it_interp=Interp.begin();it_interp!=Interp.end();it_interp++)
						//	{
						//		std::cout<<"<"<<(*it_interp)->plane_cur->index<<","<<(*it_interp)->plane_ref->index<<">, ";
						//	}
						//	std::cout<<std::endl;
						//}
						interp_length=0;
						while(interp_node2!=root)
						//for(std::vector<Node_InterpTree*>::iterator it_interp=Interp.begin();it_interp!=Interp.end();it_interp++)
						{
							if(!flag_consistent)
								break;
							//interp_node2=*it_interp;
							if(interp_node2->plane_ref==tmp_node->plane_ref)
							{
								flag_consistent=false;
								break;
							}
							if(!consistent_2(tmp_node,interp_node2))
							{
								flag_consistent=false;
								break;
							}
							else
							{
								interp_node3=node_attach;
								while(interp_node3!=root)
								//for(std::vector<Node_InterpTree*>::iterator ite_interp=Interp.begin();ite_interp!=Interp.end();ite_interp++)
								{
									if(interp_node3==interp_node2)
									{
										interp_node3=interp_node3->parent;
										continue;
									}
									//interp_node3=*ite_interp;
									if(!consistent_3(tmp_node,interp_node2,interp_node3))
									{
										flag_consistent=false;
										break;
									}
									interp_node3=interp_node3->parent;
								}
							}
							interp_length++;
							interp_node2=interp_node2->parent;
						}
						if(flag_consistent)
						{
							if(debug)
							{
								std::cout<<"inset <"<<tmp_node->plane_cur->index<<","<<tmp_node->plane_ref->index<<"> to <"<<node_attach->plane_cur->index<<","<<node_attach->plane_ref->index<<">"<<std::endl;
							}
							insert_node=new Node_InterpTree(*it_ref,*it_cur);
							node_attach->insertChild(insert_node);
							nodes_tmp.push_back(insert_node);
							//Interp.push_back(insert_node);
							if(interp_length>interp_length_max)
							{
								interp_length_max=interp_length;
								leaf_max_interp=insert_node;
							}
						}
					}
				}
				for(int i_node=0;i_node<nodes_tmp.size();i_node++)
				{
					nodes.push_back(nodes_tmp[i_node]);
				}
				nodes_tmp.clear();
			}
		}
		if(debug)
		{
			std::cout<<"final interp:"<<std::endl;
			node_attach=leaf_max_interp;
			while(node_attach!=root)
			{
				std::cout<<"<"<<node_attach->plane_cur->index<<","<<node_attach->plane_ref->index<<">,";
				node_attach=node_attach->parent;
			}
			std::cout<<std::endl;
		}
		delete tmp_node;
		return true;
	}

	bool InterpTree::isLeaf(Node_InterpTree *node)
	{
		if(node->parent!=NULL && node->children.size()==0)
			return true;
		else
			return false;
	}

	bool InterpTree::consistent_1(Node_InterpTree *node)
	{
		Eigen::Vector3d tmp;
		tmp=node->plane_cur->avg_rgb-node->plane_ref->avg_rgb;
		if(debug)
		{
			std::cout<<"\tconsistent_1: <"<<node->plane_cur->index<<","<<node->plane_ref->index<<"> "<<tmp.norm()<<std::endl;
		}
		if(tmp.norm()<thres_color)
			return true;
		else
			return false;
	}

	bool InterpTree::consistent_2(Node_InterpTree *node1, Node_InterpTree *node2)
	{
		//std::cout<<"consistent_2:"<<std::endl;
		double cos_angle_cur, cos_angle_ref, angle_cur, angle_ref, dist_cur, dist_ref;
		cos_angle_cur=node1->plane_cur->normal.transpose()*node2->plane_cur->normal;
		cos_angle_ref=node1->plane_ref->normal.transpose()*node2->plane_ref->normal;
		angle_cur=acos(cos_angle_cur);
		angle_ref=acos(cos_angle_ref);
		if(angle_cur<thres_delta_angle)
		{
			dist_cur=abs(node1->plane_cur->d-node2->plane_cur->d);
		}
		else
		{
			dist_cur=0;
		}
		if(angle_ref<thres_delta_angle)
		{
			dist_ref=abs(node1->plane_ref->d-node2->plane_ref->d);
		}
		else
		{
			dist_ref=0;
		}
		if(debug)
		{
			std::cout<<"\tconsistent_2: <"<<node1->plane_cur->index<<","<<node1->plane_ref->index<<"> "<<angle_cur<<","<<dist_cur<<"; <"<<node2->plane_cur->index<<","<<node2->plane_ref->index<<"> "<<angle_ref<<","<<dist_ref<<std::endl;
		}
		if(abs(angle_cur-angle_ref)<thres_delta_angle && abs(dist_cur-dist_ref)<thres_delta_d)
			return true;
		else
			return false;
	}

	bool InterpTree::consistent_3(Node_InterpTree *node1, Node_InterpTree *node2, Node_InterpTree *node3)
	{
		return true;
	}

	PlaneMatching_Interptree::PlaneMatching_Interptree()
	{
		interp_tree = new InterpTree;
	}

	PlaneMatching_Interptree::~PlaneMatching_Interptree()
	{
	}

	void PlaneMatching_Interptree::Match(std::vector<PlanePair> &matched_planes)
	{
		//std::cout<<"in the match:"<<interp_tree->getRoot()->children.size()<<std::endl;
		interp_tree->Construct(planes_ref, planes_cur);
		interp_tree->setDebug(debug);
		PlanePair tmp_plane_pair;
		Node_InterpTree *node_tmp=interp_tree->getMaxInterp();
		matched_planes.clear();
		while(node_tmp!=interp_tree->getRoot())
		{
			tmp_plane_pair.cur=node_tmp->plane_cur;
			tmp_plane_pair.ref=node_tmp->plane_ref;
			node_tmp=node_tmp->parent;
			matched_planes.push_back(tmp_plane_pair);
		}
	}

	//void PlaneMatching_Interptree::depthFirstSearch()
	//{
	//	//std::cout<<"node num:"<<interp_tree->getNodeNum()<<std::endl;
	//	std::stack<Node_InterpTree*> nodeStack;
	//	nodeStack.push(interp_tree->getRoot());
	//	Node_InterpTree *node;
	//	std::cout<<"depthFirstSearch: ";
	//	while(!nodeStack.empty())
	//	{
	//		//std::cout<<"here"<<std::endl;
	//		node = nodeStack.top();
	//		nodeStack.pop();
	//		for(std::vector<Node_InterpTree*>::iterator it=node->children.begin();it!=node->children.end();++it)
	//		{
	//			nodeStack.push(*it);
	//		}
	//		if(node!=interp_tree->getRoot())
	//		{
	//			std::cout<<"<"<<node->plane_cur->index<<","<<node->plane_ref->index<<">("<<node->layer<<"); ";
	//		}
	//	}
	//	std::cout<<std::endl;
	//}

	//void PlaneMatching_Interptree::breadthFirstTravel()
	//{
	//	std::queue<Node_InterpTree*> nodeQueue;
	//	nodeQueue.push(interp_tree->getRoot());
	//	Node_InterpTree *node;
	//	std::cout<<"breadthFirstTravel:";
	//	while(!nodeQueue.empty())
	//	{
	//		node = nodeQueue.front();
	//		nodeQueue.pop();
	//		for(std::vector<Node_InterpTree*>::iterator it=node->children.begin();it!=node->children.end();++it)
	//		{
	//			nodeQueue.push(*it);
	//		}
	//		if(node!=interp_tree->getRoot())
	//		{
	//			std::cout<<"<"<<node->plane_cur->index<<","<<node->plane_ref->index<<">; ";
	//		}
	//	}
	//	std::cout<<std::endl;
	//}
}


