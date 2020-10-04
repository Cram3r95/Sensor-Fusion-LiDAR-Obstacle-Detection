/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{ 
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert_aux(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node == NULL) // The tree is empty or pointing an ending leaf. End recursion 
		{
			*node = new Node(point,id); 
		}
		else 
		{
			int dimensions = 3; // X, Y, Z dimensions
			uint cd = depth%dimensions; // 0, 1, 2

			if (point[cd] < (*node)->point[cd])
			{
				insert_aux(&((*node)->left),depth+1,point,id); // depth+1 to traverse down into the tree
			}
			else
			{
				insert_aux(&((*node)->right),depth+1,point,id);
			}	
		}
		
	}
	void insert(std::vector<float> point, int id)
	{ 
		insert_aux(&root,0,point,id);
	}

	// Carlos Gomez Huelamo - My functions

	void search_aux_3D(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL) // If tree is not empty
		{
			if    ((node->point[0] >= (target[0]-distanceTol)) && (node->point[0] < (target[0]+distanceTol))     // X
			    && (node->point[1] >= (target[1]-distanceTol)) && (node->point[1] < (target[1]+distanceTol))
				&& (node->point[2] >= (target[2]-distanceTol)) && (node->point[2] < (target[2]+distanceTol)))  // Y
			{
				// Calculate 3D Euclidean Distance

				float d = sqrt(pow(node->point[0]-target[0],2)+pow(node->point[1]-target[1],2)+pow(node->point[2]-target[2],2)); 

				//std::cout<<"Distance: "<<d<<std::endl;

				if (d <= distanceTol) // Sphere around the target point
				{
					ids.push_back(node->id);
				}
			}

			// Check accross boundary (the current point we are trying to associate to target point)
			// did not lie on the bounding box region, so call again recursively

			if ((target[depth%2]-distanceTol) < node->point[depth%2]) // Left (x axis) or down (y axis)
			{
				search_aux_3D(target,node->left,depth+1,distanceTol,ids); 
			}
			if ((target[depth%2]+distanceTol) > node->point[depth%2])
			{
				search_aux_3D(target,node->right,depth+1,distanceTol,ids);
			}
		}
	}

	// Carlos Gomez Huelamo - End of My funtions
/*
	void search_aux(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			if    ((node->point[0] >= (target[0]-distanceTol)) && (node->point[0] < (target[0]+distanceTol))
			    && (node->point[1] >= (target[1]-distanceTol)) && (node->point[1] < (target[1]+distanceTol)))
			{
				// 2D Euclidean distance

				float d = sqrt(pow(node->point[0]-target[0],2)+pow(node->point[1]-target[1],2)); 

				if (d <= distanceTol) // Circle around the target point
				{
					ids.push_back(node->id);
				}
			}

			// Check accross boundary (the current point we are trying to associate to target point)
			// did not lie on the bounding box region, so call again recursively

			if ((target[depth%2]-distanceTol) < node->point[depth%2]) // Left (x axis) or down (y axis))
			{
				search_aux(target,node->left,depth+1,distanceTol,ids); 
			}
			if ((target[depth%2]+distanceTol) > node->point[depth%2])
			{
				search_aux(target,node->right,depth+1,distanceTol,ids);
			}
		}
	}
*/
	// Returns indices of nearby points w.r.t target point according to a certain tolerance
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		//search_aux(target, root, 0, distanceTol, ids); 
		search_aux_3D(target, root, 0, distanceTol, ids); 

		return ids;
	}
};




