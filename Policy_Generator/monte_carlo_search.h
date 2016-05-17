#ifndef MONTE_CARLO_SEARCH_H
#define MONTE_CARLO_SEARCH_H

/*	class MonteCarloSearch
 *		heuristic search on occupancy grid map for shirtest path
 */

#include "map_standards.h"
#include "occupancy_grid_map.h"
#include "monte_carlo_search_standards.h"

namespace MONTE_CARLO_SEARCH
{
	// Parameters for search
	struct PARAMETERS
	{
		unsigned int NumSearches;		// How often should the search be done
	};

}

// Extra Data stored in tree nodes can be added with T
template<class T = void>
class MonteCarloSearch
{
	public:
		MonteCarloSearch();

		int PerformMonteCarlo(const MONTE_CARLO_SEARCH::PARAMETERS &Parameters, const OccupancyGridMap &OGMap, const POS_2D &StartPos, const POS_2D &Destination);		// Search through map given certain parameters, a start position and a destination

	private:


};

// Must be included here due to template
#include "monte_carlo_search.cpp"

#endif // MONTE_CARLO_SEARCH_H
