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
	typedef float ACTION_COST_TYPE;		// Type of the action costs (move/observe action)

	typedef float NODE_VALUE_TYPE;
	typedef float NODE_CERTAINTY_TYPE;
	typedef float NODE_EXPECTEDLENGTH_TYPE;

	struct NODE_DATA
	{
		NODE_EXPECTEDLENGTH_TYPE ExpectedLength;	// Expected Length to reach destination from this node
		NODE_CERTAINTY_TYPE		 Certainty;			// Expected certainty of reaching destination from this node

		ACTION_COST_TYPE		CostToDest;		// Cost to reach destination from this node

		NODE_VALUE_TYPE NodeValue;		// Value of this node
		int				NumVisits;		// Number of visits to this node

		POS_2D NewCell;		// New position that was observed/moved to
		bool Observe;		// 1: Observation action, 0: Move action
		bool OccupiedCell;	// 1: New observed cell is occupied, 0: New observed cell is free

		NODE_DATA() {}
		NODE_DATA(const NODE_EXPECTEDLENGTH_TYPE &_ExpectedLength, const NODE_CERTAINTY_TYPE &_Certainty, const ACTION_COST_TYPE &_CostToDest, const NODE_VALUE_TYPE &_NodeValue, const int &_NumVisits, const POS_2D &_NewCell, const bool _Observe, const bool _OccupiedCell) : ExpectedLength(_ExpectedLength), Certainty(_Certainty), CostToDest(_CostToDest), NodeValue(_NodeValue), NumVisits(_NumVisits), NewCell(_NewCell), Observe(_Observe), OccupiedCell(_OccupiedCell) {}
	};
	const NODE_DATA NODE_DATA_EMPTY(0, 0, 0, 0, 0, POS_2D(0,0), 0, 0);

	typedef TreeNode<NODE_DATA> TREE_NODE;
	typedef TreeClass<NODE_DATA> TREE_CLASS;

	// compare for sort function
	class NODE_LENGTH_COMPARE
	{
		public:
		bool operator()(const TREE_NODE *const i, const TREE_NODE *const j) const { return (i->GetData().ExpectedLength < j->GetData().ExpectedLength); }
	};
}

class MonteCarloOption
{
	public:
		typedef MONTE_CARLO_OPTION::NODE_VALUE_TYPE MCO_NODE_VALUE_TYPE;
		typedef MONTE_CARLO_OPTION::NODE_DATA MCO_NODE_DATA;
		typedef MONTE_CARLO_OPTION::TREE_NODE MCO_TREE_NODE;
		typedef MONTE_CARLO_OPTION::TREE_CLASS MCO_TREE_CLASS;

		MonteCarloOption();

		int PerformMonteCarlo(const OccupancyGridMap &OGMap, const POS_2D &StartPos, const POS_2D &Destination);

	private:

		// Temporary data necessary for simulation
		const Map<OGM_TYPE>		*_pTmpProbMap;	// Temporary probability map
		Map<OGM_LOG_TYPE>		_TmpDStarMap;	// Temporary map with D*
		Map<OGM_LOG_TYPE>		_TmpLogMap;		// Temporary Log map of probabilities
		Map<unsigned int>		_TmpVisitMap;	// Temporary map for visits
		const POS_2D			*_pLastPos;		// Pointer to parent position
		POS_2D					_DestPosition;	// Position that should be reached
		const MCO_TREE_NODE		*_pBestDestNode;	// Node that reaches the best destination

		MONTE_CARLO_OPTION::ACTION_COST_TYPE _ObservationCost;		// Cost of observing
		MONTE_CARLO_OPTION::ACTION_COST_TYPE _MoveCost;				// Cost of moving
		const MONTE_CARLO_OPTION::NODE_LENGTH_COMPARE _CompareClass;		// Comparison for backtrack

		// Functions for Monte Carlo
		static int Selection(MCO_TREE_CLASS &Tree, MCO_TREE_NODE **SelectedNode, void *ExtraData);
		static int Expansion(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToExpand, void *ExtraData);
		static int Simulation(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE *ParentOfNodesToSimulate, void *ExtraData);
		static int Backtrack(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE *PaerntOfLeafsToBacktrack, void *ExtraData);

		// Simulation functions
		int SimulateNode(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToSimulate);		// Simulate given node
		int SimulateNode_MaxReliability(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToSimulate);		// Simulate node by finding most reliable path

		int CalculateMaxReliability(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &StartNode);

		void CalculateNodeValueFromSimulation(const MCO_NODE_DATA &NodeData, MONTE_CARLO_OPTION::NODE_VALUE_TYPE &Value);		// Calculates node value using given node data
		void CalculateNodeValueFromBacktrack(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &CurBacktrackNode);

		void UpdateTmpDataWithNodeData(const MCO_NODE_DATA &NodeData);		// Updates tmpMaps and lastposition with node data
		void ResetTmpDataWithNodeData(const MCO_NODE_DATA &NodeData);		// Reset tmpMaps and lastposition to remove data from node data
};

#endif // MONTECARLOOPTION_H
