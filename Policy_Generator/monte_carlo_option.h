#ifndef MONTECARLOOPTION_H
#define MONTECARLOOPTION_H

/*	class MonteCarloOption
 *		test Monte Carlo Algorithm
 */

#include "map_standards.h"
#include "occupancy_grid_map.h"
#include "monte_carlo.h"

namespace MONTE_CARLO_OPTION
{
	typedef double NODE_VALUE_TYPE;

	struct NODE_DATA
	{
		double ExpectedLength;		// Expected Length to reach destination from this node
		double Certainty;			// Expected certainty of reaching destination from this node

		NODE_VALUE_TYPE NodeValue;		// Value of this node
		int				NumVisits;		// Number of visits to this node

		POS_2D NewCell;		// New position that was observed/moved to
		bool Observe;		// 1: Observation action, 0: Move action
		bool OccupiedCell;	// 1: New observed cell is occupied, 0: New observed cell is free

		//NODE_DATA() {}
		NODE_DATA(const double &_ExpectedLength, const double &_Certainty, const NODE_VALUE_TYPE &_NodeValue, const int &_NumVisits, const POS_2D &_NewCell, const bool _Observe, const bool _OccupiedCell) : NumVisits(_NumVisits), NodeValue(_NodeValue), ExpectedLength(_ExpectedLength), Certainty(_Certainty), NewCell(_NewCell), Observe(_Observe), OccupiedCell(_OccupiedCell) {}
	};
	const NODE_DATA NODE_DATA_EMPTY(0, 0, 0, 0, POS_2D(0,0), 0, 0);

	typedef TreeNode<NODE_DATA> TREE_NODE;
	typedef TreeClass<NODE_DATA> TREE_CLASS;


	//	struct MCO_MAP_DATA
	//	{
	//		unsigned int	NumVisits;				// Number of times this point was visited
	//		OGM_TYPE		OccupancyProbability;	// Probability that this cell is occupied

	//		MCO_MAP_DATA(const unsigned int &_NumVisits, const OGM_TYPE &_OccupancyProbability):NumVisits(_NumVisits), OccupancyProbability(_OccupancyProbability) {}

	//		// Equals operator for copying data from original map to this one
	//		MCO_MAP_DATA operator=(OGM_TYPE T) { return MCO_MAP_DATA(this->NumVisits,T); }
	//	};
}

class MonteCarloOption
{
	public:
		typedef MONTE_CARLO_OPTION::NODE_VALUE_TYPE NODE_VALUE_TYPE;
		typedef MONTE_CARLO_OPTION::NODE_DATA NODE_DATA;
		typedef MONTE_CARLO_OPTION::TREE_NODE TREE_NODE;
		typedef MONTE_CARLO_OPTION::TREE_CLASS TREE_CLASS;
//		typedef MONTE_CARLO_OPTION::MCO_MAP_DATA MCO_MAP_DATA;

		MonteCarloOption();

		int PerformMonteCarlo(const OccupancyGridMap &OGMap);

	private:

		MonteCarlo<NODE_DATA> _Algorithm;		// Algorithm that actually performs Monte Carlo

		Map<OGM_TYPE>			_TmpProbMap;	// Temporary probability map
		Map<unsigned int>		_TmpVisitMap;	// Temporary map for visits

		// Functions for Monte Carlo
		static int Selection(const TREE_CLASS &Tree, const TREE_NODE *SelectedNode, void *ExtraData);
		static int Expansion(const TREE_CLASS &Tree, TREE_NODE &NodeToExpand, void *ExtraData);
		static int Simulation(const TREE_CLASS &Tree, TREE_NODE *ParentOfNodesToSimulate, void *ExtraData);
		static int Backtrack(const TREE_CLASS &Tree, TREE_NODE *LeafToBacktrack, void *ExtraData);
};

#endif // MONTECARLOOPTION_H
