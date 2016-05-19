#ifndef MONTECARLOSEARCHEXPANSION_H
#define MONTECARLOSEARCHEXPANSION_H

#include "monte_carlo_standards.h"

template<class T>
class MonteCarloExpansion
{
	public:
		typedef int (*ExpansionFcn)( TreeClass<T> *TreeData, TreeNode<T> *NodeToExpand);

		MonteCarloExpansion();

		void SetExpansionFcn(ExpansionFcn NewFcn) { this->_RunExpansionFcn = NewFcn; }
		void SetTree(TreeClass<T> * const NewTree) { this->_Tree = NewTree; }

		int PerformExpansion(TreeNode<T> *NodeToExpand) { return this->_RunExpansionFcn(this->Tree, NodeToExpand); }

	private:

		ExpansionFcn _RunExpansionFcn;		// Expansion function
		TreeClass<T> *_Tree;				// Tree data
};

#endif // MONTECARLOSEARCHEXPANSION_H
