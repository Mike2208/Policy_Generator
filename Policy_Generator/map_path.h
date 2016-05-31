#ifndef MAPPATH_H
#define MAPPATH_H

/* class MapPath
 *		used to store paths that a robot can follow
 */

#include "map_standards.h"

#include <vector>

class MapPath
{
	public:

		MapPath();
		MapPath(const MapPath &S);
		~MapPath();

		void Reset();											// Clears all movement orders
		void AddMoveOrder(const PATH_DATA_LOCAL &NewOrder);		// Adds one movement order to end of movementOrders

		void SetPath(const POSE &StartPose, const std::vector<PATH_DATA_LOCAL> &Movements);			// Sets the path and calculates the end position after movement
		void SetDirectPath(const POSE &StartPose, const POSE &EndPose);								// Sets start and end points and calculates the direct path

		unsigned int GetNumMovementOrders() const;			// Gets number of movements necessary to follow path
		int			 GetMovementOrder(const unsigned int &OrderNum, PATH_DATA_LOCAL &Order) const;			// Gets one movement order, returns negative value if error encountered

		POSE GetStartPose() const;			// Gets start position
		POSE GetEndPose()	const;			// Gets end position
		PATH_DATA_LOCAL GetMovementOrder(const unsigned int &OrderNum) const;				// Gets one movement order from the array

		void RecalculateEndPose();		// Calculate end position with stored Start Pose and Movement Orders

	private:

		POSE _StartPosition;									// Contains the robots start position before following the path
		POSE _EndPosition;									// Contains the robots end position after following movementOrders
		std::vector<PATH_DATA_LOCAL>	_MovementOrders;		// Contains the movement orders of the robot

		void UpdateEndPose(const PATH_DATA_LOCAL &NextMoveOrder);		// Updates end position with given move order
};

#endif // MAPPATH_H
