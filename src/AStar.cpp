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

AStar::AStar(INT gridXSize, INT gridYSize, INT gridResolutionX, INT gridResolutionY):
	m_gridXSize(gridXSize),
	m_gridYSize(gridYSize),
	m_gridResolutionX(gridResolutionX),
	m_gridResolutionY(gridResolutionY){
	m_searchGrid = std::vector<std::vector<INT>>(gridYSize*gridResolutionY, std::vector<INT>(gridXSize*gridResolutionX, 0));
}

std::vector<gridPose> AStar::search(pose start, pose end){
	std::ofstream myfile;
	myfile.open ("exploredGraph.txt");
	myfile << "x,y,cost\n";

	std::vector<gridPose> path;

	// This is complete grid search and can take a lot of time
	std::vector<std::vector<bool>> alreadyVisited(m_searchGrid.size(), std::vector<bool>(m_searchGrid[0].size(), false));
	std::vector<std::vector<gridPose>> finalparents(m_searchGrid.size(), std::vector<gridPose>(m_searchGrid[0].size(), gridPose(-1,-1)));

	std::cout<<alreadyVisited.size()<<", "<<alreadyVisited[0].size()<<"\n";
	std::priority_queue<weightedGridPose, std::vector<weightedGridPose>, std::greater<weightedGridPose>> searchQueue;
//	std::priority_queue<weightedGridPose> searchQueue;

	auto startGridPose = getGridPoseFromPose(start);
	auto endGridPose = getGridPoseFromPose(end);

	std::cout<<"startGridPose : "<<startGridPose.x<<", "<<startGridPose.y<<"\n";
	std::cout<<"endGridPose : "<<endGridPose.x<<", "<<endGridPose.y<<"\n";


//	searchQueue.push(weightedGridPose(startGridPose, startGridPose, 0.0 + heuristic(startGridPose, endGridPose)));
	searchQueue.push(weightedGridPose(startGridPose, startGridPose, 0.0, heuristic(startGridPose, endGridPose)));


	bool pathFound = false;
//	int counter = 0;
	while(!searchQueue.empty())
	{
		auto node = searchQueue.top();
//		std::cout<<searchQueue.size()<<"\t";
		searchQueue.pop();
		if(alreadyVisited[node.data.x][node.data.y]){
			continue;
		}
		myfile<<node.data.x<<", "<<node.data.y<<", "<<node.cost<<"\n";
//		std::cout<<"\n"<<++counter<<": extractedNode : "<<node.data.x<<", "<<node.data.y<<"\n";
//		std::cout<<searchQueue.size()<<"\t";
		alreadyVisited[node.data.x][node.data.y] = true;
		finalparents[node.data.x][node.data.y] = node.parent;

		if(node.data == endGridPose){
			pathFound = true;
			break;
		}

		auto neighbors = expandNeighbors(node.data);
		for(auto neighbor : neighbors){
			if(!alreadyVisited[neighbor.x][neighbor.y]){
				weightedGridPose w;
				w.data = neighbor;
				w.parent = node.data;
//				w.cost = node.cost + heuristic(w.data, node.data) + heuristic(w.data, endGridPose); // only dividing by X resolution for now
//				std::cout<<"neighbor : "<<w.data.x<<", "<<w.data.y<<", endGridPose : "<<endGridPose.x<<", "<<endGridPose.y<<"\n";
//				std::cout<<"parent cost : "<<node.cost<<", heuristic cost : "<<heuristic(w.data, endGridPose)<<"\n";
				w.cost = node.cost + 1;
				w.heuristicCost = w.cost + heuristic(w.data, endGridPose); // only dividing by X resolution for now
				searchQueue.push(w);
			}
		}
//		std::cout<<searchQueue.size()<<"\n";
	}

	if(pathFound){

		while(endGridPose != startGridPose){
			path.push_back(endGridPose);
			endGridPose = finalparents[endGridPose.x][endGridPose.y];
		}
		path.push_back(startGridPose);
	}
	return path;
}

// Return all the valid neighbors, removes neighbors covered by obstacles and obstacles outside the search grid.
std::vector<gridPose> AStar::expandNeighbors(gridPose location){
	std::vector<gridPose> neighbors;
	neighbors.reserve(4);
	// East neighbor
	{
		INT xn = location.x + 1;
		INT yn = location.y;
		gridPose neighbor = gridPose(xn, yn);
		if(isInsideGrid(neighbor)){
			neighbors.push_back(neighbor);
		}
	}

	// West neighbor
	{
		INT xn = location.x - 1;
		INT yn = location.y;
		gridPose neighbor = gridPose(xn, yn);
		if(isInsideGrid(neighbor)){
			neighbors.push_back(neighbor);
		}
	}

	// North neighbor
	{
		INT xn = location.x;
		INT yn = location.y + 1;
		gridPose neighbor = gridPose(xn, yn);
		if(isInsideGrid(neighbor)){
			neighbors.push_back(neighbor);
		}
	}

	// South neighbor
	{
		INT xn = location.x;
		INT yn = location.y - 1;
		gridPose neighbor = gridPose(xn, yn);
		if(isInsideGrid(neighbor)){
			neighbors.push_back(neighbor);
		}
	}

//	// North East neighbor
//	{
//		INT xn = location.x + 1;
//		INT yn = location.y + 1;
//		gridPose neighbor = gridPose(xn, yn);
//		if(isInsideGrid(neighbor)){
//			neighbors.push_back(neighbor);
//		}
//	}
//
//	// North South neighbor
//	{
//		INT xn = location.x - 1;
//		INT yn = location.y + 1;
//		gridPose neighbor = gridPose(xn, yn);
//		if(isInsideGrid(neighbor)){
//			neighbors.push_back(neighbor);
//		}
//	}
//
//	// South East neighbor
//	{
//		INT xn = location.x + 1;
//		INT yn = location.y - 1;
//		gridPose neighbor = gridPose(xn, yn);
//		if(isInsideGrid(neighbor)){
//			neighbors.push_back(neighbor);
//		}
//	}
//
//	// South West neighbor
//	{
//		INT xn = location.x - 1;
//		INT yn = location.y - 1;
//		gridPose neighbor = gridPose(xn, yn);
//		if(isInsideGrid(neighbor)){
//			neighbors.push_back(neighbor);
//		}
//	}

	return neighbors;
}

gridPose AStar::getGridPoseFromPose(pose p){
	gridPose gp;
	gp.x = p.x * m_gridResolutionX;
	gp.y = p.y * m_gridResolutionY;
	return gp;
}

//pose AStar::getPoseFromGridPose(gridPose gp){
//	pose p;
//	p.x = (gp.x + 1) * 0.5;
//	p.y = (gp.y + 1) * 0.5;
//	return p;
//}

bool AStar::isInsideGrid(gridPose gp){
	// First two checks are also helping unsafe calls to m_searchGrid[0].size()
	return m_searchGrid.size() > 0u && m_searchGrid[0].size() > 0u &&
			gp.y >= 0 && gp.x >= 0 &&
			gp.y < static_cast<INT>(m_searchGrid.size()) &&
			gp.x < static_cast<INT>(m_searchGrid[0].size());
}

bool AStar::isCollisionFree(gridPose gp){
	return true;
}

double AStar::heuristicL2(gridPose gp, gridPose dest){
	return ((gp.x-dest.x)*(gp.x-dest.x) +
			(gp.y-dest.y)*(gp.y-dest.y));
}

double AStar::heuristicLInfinity(gridPose gp, gridPose dest){
	return (std::abs(gp.x-dest.x) +
			std::abs(gp.y-dest.y))*2;
}

double AStar::heuristic(gridPose gp, gridPose dest){
	return heuristicL2(gp, dest);
//	return 0;
//	return heuristicLInfinity(gp, dest);
}

} /* namespace mopl */
