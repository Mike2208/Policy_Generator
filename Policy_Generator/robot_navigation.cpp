#include "robot_navigation.h"

RobotNavigation::RobotNavigation()
{

}

void RobotNavigation::GetAllNextMovementPositions(const POS_2D &CurPos, std::vector<POS_2D> &NextPositions)
{
	NextPositions.resize(RobotNavigation::GetNumNextMovementPositions());

	for(unsigned int i=0; i<NextPositions.size(); i++)
	{
		GetNextMovementPosition(CurPos, i, NextPositions[i]);
	}
}

inline unsigned int RobotNavigation::GetNumNextMovementPositions()
{
	return ROBOT_NAVIGATION::NumMoveOrders;
}

POS_2D RobotNavigation::GetNextMovementPosition(const POS_2D &CurPos, const unsigned int &MoveOrder)
{
	return (CurPos + ROBOT_NAVIGATION::MoveOrders[MoveOrder]);
}

void RobotNavigation::GetNextMovementPosition(const POS_2D &CurPos, const unsigned int &MoveOrder, POS_2D &NextPos)
{
	NextPos = CurPos + ROBOT_NAVIGATION::MoveOrders[MoveOrder];
}
