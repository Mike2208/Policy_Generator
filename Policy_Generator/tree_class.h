#ifndef TREECLASS_H
#define TREECLASS_H

#include "tree_node.h"

template<class NodeData>
class TreeClass
{
	public:

		TreeClass() : _Root() {}

		TreeNode<NodeData> *GetRoot() { return (&this->_Root); }

		void ResetTree(const NodeData &RootData)
		{
			this->_Root.ResetChildren();
			this->_Root.SetData(RootData);
			this->_Root.SetParent(0);
		 }

	private:

		TreeNode<NodeData> _Root;
};

#endif // TREECLASS_H
