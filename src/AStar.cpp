/*
 *  AStar.cpp
 *
 *  Created on: Sep 27, 2020
 *  Author: Manish Sharma
 */

#include <iostream>
#include "AStar.h"
#include <cmath>
#include <map>
#include <unordered_set>
#include <queue>
#include <fstream>

namespace mopl {

AStar::AStar(INT gridSSize, INT gridDSize, INT gridResolutionS, INT gridResolutionD):
	m_gridSSize(gridSSize),
	m_gridDSize(gridDSize),
	m_gridResolutionS(gridResolutionS),
	m_gridResolutionD(gridResolutionD){
	m_searchGrid = std::vector<std::vector<INT>>(m_gridDSize*m_gridResolutionD, std::vector<INT>(m_gridSSize*m_gridResolutionS, 0));
}

std::vector<pose> AStar::search(pose start, pose end){
	std::ofstream myfile;
	myfile.open ("exploredGraph.txt");
	myfile << "x,y,cost\n";

	std::vector<pose> path;

	// This is complete grid search and can take a lot of time
	std::vector<std::vector<bool>> alreadyVisited(m_searchGrid.size(), std::vector<bool>(m_searchGrid[0].size(), false));
	std::vector<std::vector<gridPose>> finalparents(m_searchGrid.size(), std::vector<gridPose>(m_searchGrid[0].size(), gridPose(-1,-1)));

	std::cout<<alreadyVisited.size()<<", "<<alreadyVisited[0].size()<<"\n";
	std::priority_queue<weightedGridPose, std::vector<weightedGridPose>, std::greater<weightedGridPose>> searchQueue;
//	std::priority_queue<weightedGridPose> searchQueue;

	auto startGridPose = getGridPoseFromPose(start);
	auto endGridPose = getGridPoseFromPose(end);

	std::cout<<"startGridPose : "<<startGridPose.s<<", "<<startGridPose.d<<"\n";
	std::cout<<"endGridPose : "<<endGridPose.s<<", "<<endGridPose.d<<"\n";

	searchQueue.push(weightedGridPose(startGridPose, startGridPose, 0.0, heuristic(startGridPose, endGridPose)));


	bool pathFound = false;
	int counter = 0;
	while(!searchQueue.empty())
	{
//		if(counter > 20){
//			break;
//		}
		auto node = searchQueue.top();
//		std::cout<<searchQueue.size()<<"\t";
		searchQueue.pop();
		if(alreadyVisited[node.data.d][node.data.s]){
			continue;
		}
		myfile<<getPoseFromGridPose(node.data).x<<", "<<getPoseFromGridPose(node.data).y<<", "<<node.cost<<"\n";
		std::cout<<"\n"<<++counter<<": extractedNode : "<<node.data.s<<", "<<node.data.d<<"\n";
		alreadyVisited[node.data.d][node.data.s] = true;
		finalparents[node.data.d][node.data.s] = node.parent;

		if(node.data == endGridPose){
			std::cout<<"Path Found\n";
			pathFound = true;
			break;
		}

		auto neighbors = expandNeighbors(node.data);
		for(auto neighbor : neighbors){
			if(!alreadyVisited[neighbor.d][neighbor.s]){
				weightedGridPose w;
				w.data = neighbor;
				w.parent = node.data;
//				w.cost = node.cost + heuristic(w.data, node.data) + heuristic(w.data, endGridPose); // only dividing by X resolution for now
				std::cout<<"neighbor : "<<w.data.s<<", "<<w.data.d<<", endGridPose : "<<endGridPose.s<<", "<<endGridPose.d<<"\n";
				std::cout<<"parent cost : "<<node.cost<<", heuristic cost : "<<heuristic(w.data, endGridPose)<<"\n";
				w.cost = node.cost + 1;
				w.heuristicCost = w.cost + heuristic(w.data, endGridPose); // only dividing by X resolution for now
				searchQueue.push(w);
			}else{

			}
		}
//		std::cout<<searchQueue.size()<<"\n";
	}

	if(pathFound){
		while(endGridPose != startGridPose){
			path.push_back(getPoseFromGridPose(endGridPose));
			endGridPose = finalparents[endGridPose.d][endGridPose.s];
		}
		path.push_back(getPoseFromGridPose(startGridPose));
	}else{
		std::cout<<"path not found\n";
	}
	return path;
}

// Return all the valid neighbors, removes neighbors covered by obstacles and obstacles outside the search grid.
std::vector<gridPose> AStar::expandNeighbors(gridPose location){
	std::vector<gridPose> neighbors;
	neighbors.reserve(4);
	// East neighbor
	{
		INT xn = location.s;
		INT yn = location.d + 1;
		gridPose neighbor = gridPose(xn, yn);
		if(isInsideGrid(neighbor)){
			neighbors.push_back(neighbor);
		}
	}

	// West neighbor
	{
		INT xn = location.s;
		INT yn = location.d - 1;
		gridPose neighbor = gridPose(xn, yn);
		if(isInsideGrid(neighbor)){
			neighbors.push_back(neighbor);
		}
	}

	// North neighbor
	{
		INT xn = location.s + 1;
		INT yn = location.d;
		gridPose neighbor = gridPose(xn, yn);
		if(isInsideGrid(neighbor)){
			neighbors.push_back(neighbor);
		}
	}

	// South neighbor
	{
		INT xn = location.s - 1;
		INT yn = location.d;
		gridPose neighbor = gridPose(xn, yn);
		if(isInsideGrid(neighbor)){
			neighbors.push_back(neighbor);
		}
	}

	return neighbors;
}

gridPose AStar::getGridPoseFromPose(pose p){
	gridPose gp;
	gp.s = p.x * m_gridResolutionS;
	gp.d = p.y * m_gridResolutionD;
	return gp;
}

pose AStar::getPoseFromGridPose(gridPose gp){
	pose p;
//	m_gridResolutionS(gridResolutionS),
//	m_gridResolutionD(gridResolutionD){
	p.x = 0.5 + static_cast<double>(gp.s) / m_gridResolutionS;
	p.y = 0.5 + static_cast<double>(gp.d) / m_gridResolutionD;
	return p;
}

bool AStar::isInsideGrid(gridPose gp){
	// First two checks are also helping unsafe calls to m_searchGrid[0].size()
	bool retVal =  m_searchGrid.size() > 0u && m_searchGrid[0].size() > 0u &&
			gp.d >= 0 && gp.s >= 0 &&
			gp.d < static_cast<INT>(m_searchGrid.size()) &&
			gp.s < static_cast<INT>(m_searchGrid[0].size());
	std::cout<<"returning "<<retVal<<" for gp : "<<gp.s<<", "<<gp.d<<"\n";
	return retVal;
}

bool AStar::isCollisionFree(gridPose gp){
	return true;
}

double AStar::heuristicL2(gridPose gp, gridPose dest){
	return ((gp.s-dest.s)*(gp.s-dest.s) +
			(gp.d-dest.d)*(gp.d-dest.d));
}

double AStar::heuristicLInfinity(gridPose gp, gridPose dest){
	return (std::abs(gp.s-dest.s) +
			std::abs(gp.d-dest.d));
}

double AStar::heuristic(gridPose gp, gridPose dest){
	return heuristicL2(gp, dest);
//	return 0;
//	return heuristicLInfinity(gp, dest);
}

} /* namespace mopl */
