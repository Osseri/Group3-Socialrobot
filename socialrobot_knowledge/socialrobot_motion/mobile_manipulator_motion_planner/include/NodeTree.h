#ifndef __NODETREE__
#define __NODETREE__

#include "RRTNode.h"

class NodeTree
{
public:
	std::vector<RRTNode*> _nodes;
	RRTNode* getNode(int index) { return _nodes[index]; }
	void addNode(RRTNode &node) { _nodes.push_back(&node); }
	void removeNode(int index) { _nodes.erase(_nodes.begin() + index); }
	std::vector<std::vector<double> > getPath() {
		std::vector<std::vector<double> > path;
		RRTNode* node = _nodes.back(); //start at goal
		while (node != NULL) {
			path.push_back(node->getConfiguration());
			node = node->getParent();
		}

		std::reverse(path.begin(), path.end());
		return path;
	}
	std::vector<RRTNode*> *getTree() {
		return &_nodes;
	}

	RRTNode* getNearest(std::vector<double> &robot, std::vector<double> &config, std::vector<double> &w) {
		int min = 0;
		double mindist = _nodes[0]->getDistance(robot, config, w);
		for (int i = 1; i< _nodes.size(); i++) { // Ignoring index 0 as it is preselected
			double current = _nodes[i]->getDistance(robot, config, w);
			if (current < mindist) {
				min = i;
				mindist = current;
			}
		}
		return getNode(min);
	}
};
#endif