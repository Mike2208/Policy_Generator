#ifndef MONTECARLOSEARCHSELECTION_H
#define MONTECARLOSEARCHSELECTION_H

/*	class MonteCarloSearchSelection
 *		performs selection stage of monte carlo search
 *
 */

#include "tree_class.h"
#include "monte_carlo_standards.h"

namespace MONTE_CARLO_SELECTION
{
	// UCL Selection
	const float UCL_COEFFICIENT = 0.7;			// Coefficient used in OCL analysis ( tuned experimentally )
	typedef MONTE_CARLO_STANDARDS::TREE_DATA<char> UCL_TREE_DATA;
	typedef MONTE_CARLO_STANDARDS::MONTE_CARLO_TREE<char>		UCL_TREE_CLASS;
	typedef MONTE_CARLO_STANDARDS::MONTE_CARLO_TREE_NODE<char>  UCL_TREE_NODE;
	int UCL_Function(const UCL_TREE_CLASS &Tree, UCL_TREE_NODE **SelectedNode);
}

template<class T>
class MonteCarloSelection
{
	public:
		typedef int (*SelectionFcn)(const TreeClass<T> &Tree, TreeNode<T> **SelectedNode);

		MonteCarloSelection() : _Tree(0) {}
		MonteCarloSelection(const MonteCarloSelection &S);

		void SetSelectionFcn(SelectionFcn NewFcn) { this->_ReturnSelectedNode = NewFcn; }			// Function for selecting next node to simulate
		void SetTreeData(const TreeClass<T> *SearchTree) { this->_Tree = SearchTree; }

		int PerformSelection(TreeNode<T> **SelectedNode) { return this->_ReturnSelectedNode(this->_Tree, SelectedNode); }			// Returns selected node on which to run simulations

	private:

		SelectionFcn		_ReturnSelectedNode;		// Pointer to Selection function
		const TreeClass<T>	*_Tree;						// Pointer to tree data

};

#endif // MONTECARLOSEARCHSELECTION_H
