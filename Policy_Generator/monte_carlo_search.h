#ifndef MONTE_CARLO_SEARCH_H
#define MONTE_CARLO_SEARCH_H

/*	class MonteCarloSearch
 *		heuristic search on occupancy grid map for shirtest path
 */

#include "map_standards.h"
#include "occupancy_grid_map.h"

namespace MONTE_CARLO_SEARCH
{
	// Parameters for search
	struct PARAMETERS
	{
		unsigned int NumSearches;		// How often should the search be done
	};

	// Data stored in tree nodes
	struct TREE_DATA
	{
			int Wins, Losses;			// Wins and Losses of this node
			POS_2D Position;			// Position reached with this node
			POS_2D PositionRevealed;	// Position that was observed in this node ( can be same as Position )
			bool   RevealedPosOccupied;	// Was the revealed position occupied

			TREE_DATA() : Wins(0), Losses(0), RevealedPosOccupied(0) {}
	};
}

class MonteCarloSearch
{
	public:
		MonteCarloSearch();

		static int PerformMonteCarlo(const MONTE_CARLO_SEARCH::PARAMETERS &Parameters, const OccupancyGridMap &OGMap, const POS_2D &StartPos, const POS_2D &Destination);		// Search through map given certain parameters, a start position and a destination
};

#endif // MONTE_CARLO_SEARCH_H
