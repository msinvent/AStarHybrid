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
	gridPose(INT xp, INT yp){
		x = xp;
		y = yp;
	}
	bool operator ==(const gridPose& rhs){
		return this->x == rhs.x && this->y == rhs.y;
	}
	bool operator !=(const gridPose& rhs){
		return !(*this == rhs);
	}
	INT x;
	INT y;
};

class AStar {
public:
	AStar(INT gridXSize, INT gridYSize, INT gridResolutionX = 1, INT gridResolutionY = 1);
	std::vector<gridPose> search(pose start, pose end);
	std::vector<gridPose> expandNeighbors(gridPose location);
	gridPose getGridPoseFromPose(pose p);
//	pose getPoseFromGridPose(gridPose gp);
	bool isInsideGrid(gridPose gp);
	bool isCollisionFree(gridPose gp);
	double heuristicL2(gridPose gp, gridPose dest);
	double heuristicLInfinity(gridPose gp, gridPose dest);
	double heuristic(gridPose gp, gridPose dest);

	INT getGridXSize(){
		return m_gridXSize;
	}

	INT getGridYSize(){
		return m_gridYSize;
	}

	INT getGridResolutionX(){
		return m_gridResolutionX;
	}

	INT getGridResolutionY(){
		return m_gridResolutionY;
	}

	const std::vector<std::vector<INT>>& getSearchGrid(){
		return m_searchGrid;
	}

private:
	INT m_gridXSize;
	INT m_gridYSize;
	INT m_gridResolutionX;
	INT m_gridResolutionY;
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
