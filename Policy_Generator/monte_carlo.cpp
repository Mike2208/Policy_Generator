#ifndef MONTE_CARLO_CPP
#define	MONTE_CARLO_CPP

#include "monte_carlo.h"
#include "tree_class.h"

template<class T>
MonteCarlo<T>::MonteCarlo()
{

}

template<class T>
int MonteCarlo<T>::PerformMonteCarlo(const MONTE_CARLO::PARAMETERS &Parameters, const OccupancyGridMap &OGMap, const POS_2D &StartPos, const POS_2D &Destination)
{
	// Create a tree for monte carlo
	TreeClass<MONTE_CARLO_STANDARDS::TREE_DATA<T>> pathTree;
	TreeNode<MONTE_CARLO_STANDARDS::TREE_DATA<T>>	 *pCurNode;

	MONTE_CARLO_STANDARDS::TREE_DATA<T> curNodeData;

	pathTree.SetParent(NULL);		// Set parent to of top node to NULL

	pCurNode = dynamic_cast<TreeNode<MONTE_CARLO_STANDARDS::TREE_DATA<T>>*>(&pathTree);
}

#endif // MONTE_CARLO_CPP
