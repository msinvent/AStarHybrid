/*
 * AStar.h
 *
 *  Created on: Sep 27, 2020
 *      Author: manish
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include <vector>
#include <cstdint>

namespace mopl {

//	^
//	|
//	|
//	y
//	|
//	|
//(0,0)-----------X ---------->

using INT = int32_t;

class pose {
public:
	double x;
	double y;
};

// I had to define == and != operators for this class
struct gridPose {
	gridPose() = default;
	gridPose(INT sp, INT dp){
		s = sp;
		d = dp;
	}
	bool operator ==(const gridPose& rhs){
		return this->s == rhs.s && this->d == rhs.d;
	}
	bool operator !=(const gridPose& rhs){
		return !(*this == rhs);
	}
	INT s;
	INT d;
};

class AStar {
public:
	AStar(INT gridXSize, INT gridYSize, INT gridResolutionX = 1, INT gridResolutionY = 1);
	std::vector<pose> search(pose start, pose end);
	std::vector<gridPose> expandNeighbors(gridPose location);
	gridPose getGridPoseFromPose(pose p);
	pose getPoseFromGridPose(gridPose gp);
	bool isInsideGrid(gridPose gp);
	bool isCollisionFree(gridPose gp);
	double heuristicL2(gridPose gp, gridPose dest);
	double heuristicLInfinity(gridPose gp, gridPose dest);
	double heuristic(gridPose gp, gridPose dest);

	INT getGridSSize(){
		return m_gridSSize;
	}

	INT getGridDSize(){
		return m_gridDSize;
	}

	INT getGridResolutionS(){
		return m_gridResolutionS;
	}

	INT getGridResolutionD(){
		return m_gridResolutionD;
	}

	const std::vector<std::vector<INT>>& getSearchGrid(){
		return m_searchGrid;
	}

private:
	INT m_gridSSize;
	INT m_gridDSize;
	INT m_gridResolutionS;
	INT m_gridResolutionD;
	std::vector<std::vector<INT>> m_searchGrid;
};

template<typename T>
struct weightedType{
	T data;
	T parent;
	double cost{0.0};
	double heuristicCost{0.0};
	bool operator < (const weightedType rhs) const{
		return this->heuristicCost < rhs.heuristicCost;
	}
	bool operator > (const weightedType rhs) const{
		return this->heuristicCost > rhs.heuristicCost;
	}
	bool operator == (const weightedType rhs) const{
		return this->heuristicCost == rhs.heuristicCost;
	}
	weightedType(T dat, T par, double cst, double hcst):
		data(dat),
		parent(par),
		cost(cst),
		heuristicCost(hcst){};
	weightedType() = default;
};

using weightedGridPose = weightedType<gridPose>;

} /* namespace mopl */

#endif /* ASTAR_H_ */
