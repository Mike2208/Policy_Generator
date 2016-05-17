#ifndef TREECLASS_H
#define TREECLASS_H

#include "tree_node.h"

template<class NodeData>
class TreeClass
{
	public:

		TreeClass() : _Root() {}

		const TreeNode<NodeData> *GetRoot() const { return (&this->_Root); }

	private:

		TreeNode<NodeData> _Root;
};

#endif // TREECLASS_H
