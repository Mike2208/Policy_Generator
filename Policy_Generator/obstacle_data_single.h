#ifndef OBSTACLEDATASINGLE_H
#define OBSTACLEDATASINGLE_H

/*	class ObstacleDataSingle
 *		stores all positions relevant to an object, its average position, and its ID
 *
 */

#include "map_standards.h"

#include <vector>

class ObstacleDataSingle
{
	public:

		ObstacleDataSingle();

		void Reset();
		void Reset(const std::vector<POS_2D> &NewPositions, const OBSTACLE_ID &NewID);

		OBSTACLE_ID		GetID() const;
		void			SetID(const OBSTACLE_ID &NewID);

		void			AddPosition(const POS_2D &NewPositions);

		const POS_2D &operator[](unsigned int i) const { return this->_Positions[i]; }		// Bracket operator returns one of the positions of the obstacle

	private:

		std::vector<POS_2D>		_Positions;			// all positions occupied by this obstacle
		OBSTACLE_ID				_ID;				// ID of this obstacle
		POS_2D					_AveragePosition;	// average position summed up over all given pixels occupied by this element

		void RecalculateAveragePos();		// Calculates new average position
};

#endif // OBSTACLEDATASINGLE_H
