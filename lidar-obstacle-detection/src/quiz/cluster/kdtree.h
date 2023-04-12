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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		Node* to_insert = new Node(point, id);

		int depth = 0;
		Node** curr = &root;
		while (*curr != NULL) {
			int idx = depth % point.size();
			if (point[idx] < (*curr)->point[idx]) {
				// branch left
				curr = &(*curr)->left;
			} else {
				// branch right
				curr = &(*curr)->right;
			}
			depth++;
		}
		*curr = to_insert;
	}

	bool withinBox(std::vector<float>& target, std::vector<float>& src, float distanceTol) {
		for (int i = 0; i < target.size(); i++) {
			if (std::fabs(target[i] - src[i]) > distanceTol)
				return false;
		}
		return true;
	}

	bool withinDistance(std::vector<float>& target, std::vector<float>& src, float distanceTol) {
		float distance = 0;
		for (int i = 0; i < target.size(); i++) {
			distance += std::pow(target[i] - src[i], 2);
		}
		return std::sqrt(distance) <= distanceTol;
	}

	void searchHelper(std::vector<float>& target, float distanceTol, Node* curr, int depth, std::vector<int>& ids) {
		if (curr != NULL) {
			int idx = depth % target.size();
			if (withinBox(target, curr->point, distanceTol)) {
				if (withinDistance(target, curr->point, distanceTol)) {
					ids.push_back(curr->id);
				}
			} 
			
			if (target[idx] - distanceTol < curr->point[idx]) {
				searchHelper(target, distanceTol, curr->left, depth + 1, ids);
			} 
			
			if (target[idx] + distanceTol > curr->point[idx]) {
				searchHelper(target, distanceTol, curr->right, depth + 1, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, root, 0, ids);
		return ids;
	}
	

};




