#ifndef GRAPHCLASS_H
#define GRAPHCLASS_H

/*	class GraphClass
 *		used to interface with graphs
 *
 */

#include "graph_edge.h"
#include "graph_vertice.h"

#include <climits>
#include <vector>

template<class VerticeData, class EdgeData>
class GraphClass
{
	public:

		GraphClass();
		~GraphClass();

		void Reset();			// Resets graph

		void SetStartVertice(const GraphVertice<VerticeData> &StartEdge);			// Clears everything and sets this edge as start

		unsigned int GetNumEdges() const;		// Returns number of edges in graph
		unsigned int GetNumVertices() const;	// Returns number of vertices in graph

		int AddNewVertice(const VerticeData &NewVerticeData, const EdgeData &NewEdgeData, const unsigned int &ConnectVerticeID, const bool &NewEndVertice);			// Connects new vertice to end of new edge
		int ConnectVertices(const unsigned int &StartVerticeID, const unsigned int &EndVerticeID, const EdgeData &NewEdgeData);		// Connects to vertices and adds NewEdgeData to new edge

		int GetVerticeData(const unsigned int &VerticeID, VerticeData &VerticeDat) const;		// Stores vertices[VerticeNum] into &Vertice, return negative value on error
		VerticeData GetVerticeData(const unsigned int &VerticeNum) const;						// Returns vertice without error checking

		int GetEdgeData(const unsigned int &EdgeID, EdgeData &EdgeDat) const;		// Stores edges[EdgeNum] into &Edge, return negative value on error
		EdgeData GetEdgeData(const unsigned int &EdgeID) const;				// Returns edge without error checking

		int GetAdjoiningEdges(const unsigned int &VerticeID, std::vector<unsigned int> &AdjoiningEdgeIDs) const;		// Returns IDs of all connected edges
		int GetAdjoiningVerticesFromVertice(const unsigned int &VerticeID, std::vector<unsigned int> &AdjoiningVerticeIDs) const;	// Returns IDs of all connected vertices

		int GetAdjoiningVerticesFromEdge(const unsigned int &EdgeID, std::array<unsigned int, 2> &AdjoiningVertices) const;		// Returns IDs of all connected vertices

	private:

		std::vector<GraphEdge<EdgeData>>		_Edges;		// Edges of graph
		std::vector<GraphVertice<VerticeData>>	_Vertices;	// Vectors of graph

		const unsigned int NULL_POINTER = UINT_MAX;
};

#endif // GRAPHCLASS_H
