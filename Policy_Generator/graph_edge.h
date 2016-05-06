#ifndef GRAPH_EDGE_H
#define GRAPH_EDGE_H

/*	class GraphEdge
 *		helper class for GraphClass
 *		stores one edge, and the array position of the corresponding vertices
 */

#include <array>

template<class T>
class GraphEdge
{
	public:
		GraphEdge();
		~GraphEdge();

		int SetEdge(const unsigned int &Vertice1, const unsigned int &Vertice2, const T &EdgeData);			// Sets edge

		int SetEdgeData(const T &EdgeData);		// Set data of edge
		T GetEdgeData();						// Returns edge data

		int SetVertice1(const unsigned int VerticeNum);			// Sets start vertice number
		int SetVertice2(const unsigned int VerticeNum);			// Sets end vertice number

		unsigned int GetVertice1() const;						// Returns start vertice number
		unsigned int GetVertice2() const;						// Returns end vertice number

	private:

		T _Data;			// relevant data of edge;
		std::array<unsigned int, 2> _Vertices;			// Pointers to the connected vertice
};

// Include function code because of templates
#include "graph_edge.cpp"

#endif // GRAPH_EDGE_H
