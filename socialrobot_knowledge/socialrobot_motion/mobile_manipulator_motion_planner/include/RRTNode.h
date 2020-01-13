#pragma once 
#ifndef __RRTNODE__
#define __RRTNODE__

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

class RRTNode {

public:
	std::vector<double> _configuration; //Vector for the joint values in the configuration
	RRTNode *_parent;//Pointer to Parent node on RRT Tree

	RRTNode() {}
	RRTNode(std::vector<double> &configuration) : _configuration(configuration) {}
	RRTNode(std::vector<double> &configuration, RRTNode *parent) : _configuration(configuration), _parent(parent) {}
	std::vector<double> getConfiguration() {
		return _configuration;
	};
	void setConfiguration(std::vector<double> &configuration) {
		_configuration = configuration;
	};
	double getDistance(std::vector<double> &robot, std::vector<double> &config, std::vector<double> &w) {
		double accumulated = 0.0f;
		for (int i = 0; i < _configuration.size(); i++) {
			accumulated += (_configuration[i] - config[i]) * (_configuration[i] - config[i]) *w[i] * w[i];
		}
		return accumulated;
	}
	RRTNode* getParent() { return _parent; }
	
};
#endif
