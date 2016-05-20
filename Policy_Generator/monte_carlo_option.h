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
		float ExpectedLength;		// Expected Length to reach destination from this node
		float Certainty;			// Expected certainty of reaching destination from this node

		NODE_VALUE_TYPE NodeValue;		// Value of this node
		int				NumVisits;		// Number of visits to this node

		POS_2D NewCell;		// New position that was observed/moved to
		bool Observe;		// 1: Observation action, 0: Move action
		bool OccupiedCell;	// 1: New observed cell is occupied, 0: New observed cell is free

		//NODE_DATA() {}
		NODE_DATA(const float &_ExpectedLength, const float &_Certainty, const NODE_VALUE_TYPE &_NodeValue, const int &_NumVisits, const POS_2D &_NewCell, const bool _Observe, const bool _OccupiedCell) : ExpectedLength(_ExpectedLength), Certainty(_Certainty), NodeValue(_NodeValue), NumVisits(_NumVisits), NewCell(_NewCell), Observe(_Observe), OccupiedCell(_OccupiedCell) {}
	};
	const NODE_DATA NODE_DATA_EMPTY(0, 0, 0, 0, POS_2D(0,0), 0, 0);

	typedef TreeNode<NODE_DATA> TREE_NODE;
	typedef TreeClass<NODE_DATA> TREE_CLASS;

}

class MonteCarloOption
{
	public:
		typedef MONTE_CARLO_OPTION::NODE_VALUE_TYPE MCO_NODE_VALUE_TYPE;
		typedef MONTE_CARLO_OPTION::NODE_DATA MCO_NODE_DATA;
		typedef MONTE_CARLO_OPTION::TREE_NODE MCO_TREE_NODE;
		typedef MONTE_CARLO_OPTION::TREE_CLASS MCO_TREE_CLASS;
//		typedef MONTE_CARLO_OPTION::MCO_MAP_DATA MCO_MAP_DATA;

		MonteCarloOption();

		int PerformMonteCarlo(const OccupancyGridMap &OGMap);

	private:

		MonteCarlo<MCO_NODE_DATA> _Algorithm;		// Algorithm that actually performs Monte Carlo

		// Temporary data necessary for simulation
		//Map<OGM_TYPE>			_TmpProbMap;	// Temporary probability map
		Map<OGM_LOG_TYPE>		_TmpDStarMap;	// Temporary map with D*
		Map<OGM_LOG_TYPE>		_TmpLogMap;		// Temporary Log map of probabilities
		Map<unsigned int>		_TmpVisitMap;	// Temporary map for visits
		const POS_2D			*_pLastPos;		// Pointer to parent position
		POS_2D					_DestPosition;	// Position that should be reached


		// Functions for Monte Carlo
		static int Selection(const MCO_TREE_CLASS &Tree, const MCO_TREE_NODE *SelectedNode, void *ExtraData);
		static int Expansion(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToExpand, void *ExtraData);
		static int Simulation(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE *ParentOfNodesToSimulate, void *ExtraData);
		static int Backtrack(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE *LeafToBacktrack, void *ExtraData);

		// Simulation functions
		int SimulateNode_MaxReliability(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToSimulate);		// Simulate node and find most reliable path
		int CalculateMaxReliability(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &StartNode);

		void CalculateNodeValueFromSimulation(const MCO_NODE_DATA &NodeData, MONTE_CARLO_OPTION::NODE_VALUE_TYPE &Value);		// Calculates node value using given node data
};

#endif // MONTECARLOOPTION_H
