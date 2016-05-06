#ifndef GRAPH_VERTICE_H
#define GRAPH_VERTICE_H

/*	class GraphVector<T>
 *		helper class for GraphClass
 *		stores one vertice, and the array position of the connected edges
 *
 *		contains one vector with data T
 */

#include <vector>

template<class T>
class GraphVertice
{
	public:
		GraphVertice();
		~GraphVertice();

		int Reset();

		T GetVerticeData() const;		// Returns data of vertice
		int SetVerticeData(T &NewData);		// Stores data of vertice

		unsigned int GetNumEdgePointers() const;							// Gets amount of edges connected to vertice
		unsigned int GetEdgeID(const unsigned int &EdgeNum) const;			// Returns pointer to one edge
		int SetEdgeID(const unsigned int &EdgeNum, unsigned int &EdgePointer);		// Sets one edge to the given pointer
		int AddEdgeID(unsigned int &EdgePointer);									// Adds one edge

		int ClearEdgeIDs();		// Removes all edge pointers

	private:

		T _VerticeData;					// Data in this Graph
		std::vector<unsigned int> _Edges;	// Pointers to edges
};

#endif // GRAPH_VERTICE_H
