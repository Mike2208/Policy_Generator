#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <vector>

namespace TREE_NODE
{
	typedef unsigned int ID;
}

template<class T>
class TreeNode
{
	public:
		TreeNode() : _Parent(0) {}
		TreeNode(TreeNode<T> *const Parent, const T &NewData) : _Parent(Parent), _Data(NewData) {}

		void ResetChildren() { this->_Children.clear; }		// Clears all children

		unsigned int GetNumChildren() const { return this->_Children.size(); }										// Returns number of children
		const TreeNode<T> *GetChildNode(const TREE_NODE::ID &ChildID)const { return &(this->_Children[ChildID]); }	// Gets child node

		//void AddChild(const TreeNode<T> &NewChild) { this->_Children.push_back(NewChild); this->_Children[this->_Children.size()-1].SetParent(this); }
		void AddChild(const T &NewChildData) { this->_Children.push_back(TreeNode<T>(this, NewChildData)); }

		T	&GetData() const { return this->_Data; }				// Gets data of this node
		void SetData(const T &NewData) { this->_Data = NewData; }	// Sets data of this node

		TreeNode<T>	*GetParent() const { return this->_Parent; }								// Gets parent node
		void		SetParent(TreeNode<T> * const Parent) { this->_Parent = Parent; }		// Sets parent node

		bool IsLeaf() const { return (this->_Children.size() == 0 ? 0:1); }						// Returns whether this Node is a leaf

	private:

		std::vector<TreeNode<T>>	_Children;		// References to children of this node
		TreeNode<T>					*_Parent;		// Pointer to parent node
		T							_Data;			// Data stored in this node

};

#endif // TREE_NODE_H
