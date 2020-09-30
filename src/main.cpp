//============================================================================
// Name        : A_star.cpp
// Author      : Manish Sharma
// Version     : 1.0
// Description : 2D A* implementation, with L2 heuristic(optimistic guess)
//============================================================================

#include <iostream>
#include "AStar.h"
#include <fstream>

int main() {
	std::ofstream myfile;
	myfile.open ("path.txt");
	myfile << "x,y\n";

	mopl::AStar AstarObj(50, 5, 1, 5);

	// Perform basic API testing
//	std::cout<< AstarObj.getGridXSize()<<"\n";
//	std::cout<< AstarObj.getGridYSize()<<"\n";
//	std::cout<< AstarObj.getGridResolutionX()<<"\n";
//	std::cout<< AstarObj.getGridResolutionY()<<"\n";
	auto searchGrid = AstarObj.getSearchGrid();
	std::cout<<"searchGrid Size : s = "<<searchGrid[0].size()<<", d = "<<searchGrid.size()<<"\n";
//
//	mopl::gridPose gp_1(0, 0);
//	mopl::gridPose gp_2(50,0);
//	std::cout<<AstarObj.isInsideGrid(gp_1)<<"\n";
//	auto neighbors = AstarObj.expandNeighbors(gp_1);
//	for(auto neighbor : neighbors){
//		std::cout<<"neighbor : "<<neighbor.x<<", "<<neighbor.y<<"\n";
//	}

	mopl::pose p_1{0.0, 0.0};
	mopl::pose p_2{28.999999, 3.4};
	std::cout<<"start : "<<AstarObj.getGridPoseFromPose(p_1).s<<", "<<AstarObj.getGridPoseFromPose(p_1).d<<"\n";
	std::cout<<"end : "<<AstarObj.getGridPoseFromPose(p_2).s<<", "<<AstarObj.getGridPoseFromPose(p_2).d<<"\n";

	auto path = AstarObj.search(p_1, p_2);
	std::cout<<"path size : "<<path.size()<<"\n";
	for(auto pathPoint : path){
		std::cout<<"pathPoint : "<<pathPoint.x<<", "<<pathPoint.y<<"\n";
		myfile<<pathPoint.x<<", "<<pathPoint.y<<"\n";
	}

	return 0;
}
