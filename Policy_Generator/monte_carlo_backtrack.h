#ifndef MONTECARLOBACKTRAP_H
#define MONTECARLOBACKTRAP_H

/*	class MonteCarloBacktrap
 *		backtrack function
 */

#include "tree_class.h"
#include "monte_carlo_standards.h"

template<class T>
class MonteCarloBacktrack
{
	public:
		typedef int (*BacktrackFcn)( TreeClass<T> *TreeData, TreeNode<T> *NodeToExpand);

		MonteCarloBacktrack() : _Tree(0), _RunBacktrackFcn(0) {}
		MonteCarloBacktrack(const MonteCarloBacktrack &S);

		void SetBacktrackFcn(BacktrackFcn NewFcn) { this->_RunBacktrackFcn = NewFcn; }
		void SetTree(TreeClass<T> *NewTree) { this->_Tree = NewTree; }
		void SetExtraData(void *NewExtraData) { this->_ExtraData = NewExtraData; }

		int PerformBacktrack( TreeNode<T> *NodeToExpand ) { return this->_RunBacktrackFcn(this->_Tree, NodeToExpand); }

	private:

		BacktrackFcn	_RunBacktrackFcn;
		TreeClass<T>	*_Tree;
		void			*_ExtraData;
};

#endif // MONTECARLOBACKTRAP_H
