#include "monte_carlo_search.h"
#include "tree_class.h"

MonteCarloSearch::MonteCarloSearch()
{

}

int MonteCarloSearch::PerformMonteCarlo(const MONTE_CARLO_SEARCH::PARAMETERS &Parameters, const OccupancyGridMap &OGMap, const POS_2D &StartPos, const POS_2D &Destination)
{
	// Create a tree for monte carlo
	TreeClass<MONTE_CARLO_SEARCH::TREE_DATA> pathTree;
	TreeNode<MONTE_CARLO_SEARCH::TREE_DATA>	 *pCurNode;

	MONTE_CARLO_SEARCH::TREE_DATA curNodeData;

	pathTree.SetParent(NULL);		// Set parent to of top node to NULL
}
