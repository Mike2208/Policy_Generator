#ifndef ROBOTNAVIGATION_H
#define ROBOTNAVIGATION_H

/*	class RobotNavigation
 *		interface to robot navigation
 *		can be used to get navigation options (i.e. get next reachable positions)
 */

#include "map_standards.h"
#include <vector>

namespace ROBOT_NAVIGATION
{
	const unsigned int NumMoveOrders = 4;
	const POS_2D MoveOrders[NumMoveOrders] =
		{																			// make sure this is circular for other functions to work
			POS_2D(static_cast<unsigned int>(0), static_cast<unsigned int>(1)),		// move top
			POS_2D(static_cast<unsigned int>(1), static_cast<unsigned int>(0)),		// move right
			POS_2D(static_cast<unsigned int>(0), static_cast<unsigned int>(-1)),	// move bottom
			POS_2D(static_cast<unsigned int>(-1), static_cast<unsigned int>(0))		// move left
		};
}

class RobotNavigation
{
	public:

		RobotNavigation();
		RobotNavigation(const RobotNavigation &S);

		static void GetAllNextMovementPositions(const POS_2D &CurPos, std::vector<POS_2D> &NextPositions);		// Returns next movement options (this doesn't check against any map, it just states where a robot can move next)

		static unsigned int GetNumNextMovementPositions();			// Returns how many positions a robot can move to next
		static POS_2D GetNextMovementPosition(const POS_2D &CurPos, const unsigned int &MoveOrder);		// Returns a possible next positions
		static void GetNextMovementPosition(const POS_2D &CurPos, const unsigned int &MoveOrder, POS_2D &NextPos);		// Returns a possible next positions

	private:
};

#endif // ROBOTNAVIGATION_H
