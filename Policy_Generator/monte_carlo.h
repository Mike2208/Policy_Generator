#ifndef MONTE_CARLO_H
#define MONTE_CARLO_H

/*	class MonteCarloSearch
 *		heuristic search on occupancy grid map for shirtest path
 */

#include "map_standards.h"
#include "occupancy_grid_map.h"
#include "monte_carlo_standards.h"
#include "monte_carlo_selection.h"
#include "monte_carlo_expansion.h"
#include "monte_carlo_simulation.h"
#include "monte_carlo_backtrack.h"

namespace MONTE_CARLO
{
	// Parameters for search
	struct PARAMETERS
	{
		unsigned int NumSearches;		// How often should the search be done
	};

}

// Extra Data stored in tree nodes can be added with T
template<class T = void>
class MonteCarlo
{
	public:
		MonteCarlo();

		int PerformMonteCarlo(const MONTE_CARLO::PARAMETERS &Parameters, const OccupancyGridMap &OGMap, const POS_2D &StartPos, const POS_2D &Destination);		// Search through map given certain parameters, a start position and a destination

		void SetSelection(MonteCarloSelection<T>)

	private:

		MonteCarloSelection<T>	_Selection;
		MonteCarloSimulation<T>	_Simulation;
		MonteCarloExpansion<T>	_Expansion;
		MonteCarloBacktrack<T>	_Backtrack;
};

// Must be included here due to template
#include "monte_carlo.cpp"

#endif // MONTE_CARLO_H
