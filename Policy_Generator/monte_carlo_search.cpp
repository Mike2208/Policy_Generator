#ifndef MONTE_CARLO_SEARCH_CPP
#define	MONTE_CARLO_SEARCH_CPP

#include "monte_carlo_search.h"
#include "tree_class.h"

template<class T>
MonteCarloSearch<T>::MonteCarloSearch()
{

}

template<class T>
int MonteCarloSearch<T>::PerformMonteCarlo(const MONTE_CARLO_SEARCH_STANDARDS::PARAMETERS &Parameters, const OccupancyGridMap &OGMap, const POS_2D &StartPos, const POS_2D &Destination)
{
	// Create a tree for monte carlo
	TreeClass<MONTE_CARLO_SEARCH_STANDARDS::TREE_DATA<T>> pathTree;
	TreeNode<MONTE_CARLO_SEARCH_STANDARDS::TREE_DATA<T>>	 *pCurNode;

	MONTE_CARLO_SEARCH_STANDARDS::TREE_DATA<T> curNodeData;

	pathTree.SetParent(NULL);		// Set parent to of top node to NULL

	pCurNode = dynamic_cast<TreeNode<MONTE_CARLO_SEARCH_STANDARDS::TREE_DATA<T>>*>(&pathTree);
}

#endif // MONTE_CARLO_SEARCH_CPP
