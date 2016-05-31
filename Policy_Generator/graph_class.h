#ifndef GRAPHCLASS_H
#define GRAPHCLASS_H

/*	class GraphClass
 *		used to interface with graphs
 *
 */

#include "map_standards.h"

#include "graph_edge.h"
#include "graph_vertice.h"

#include <climits>
#include <vector>

template<class VerticeData, class EdgeData>
class GraphClass
{
	public:

		GraphClass();
		GraphClass(const GraphClass &S);
		~GraphClass();

		void Reset();			// Resets graph

		void SetStartVertice(const GraphVertice<VerticeData> &StartEdge);			// Clears everything and sets this edge as start

		unsigned int GetNumEdges() const;		// Returns number of edges in graph
		unsigned int GetNumVertices() const;	// Returns number of vertices in graph

		int AddNewVertice(const VerticeData &NewVerticeData, const EdgeData &NewEdgeData, const OBSTACLE_ID &ConnectVerticeID, const bool &NewEndVertice);			// Connects new vertice to end of new edge
		int ConnectVertices(const OBSTACLE_ID &StartVerticeID, const OBSTACLE_ID &EndVerticeID, const EdgeData &NewEdgeData);		// Connects to vertices and adds NewEdgeData to new edge

		int GetVerticeData(const OBSTACLE_ID &VerticeID, VerticeData &VerticeDat) const;		// Stores vertices[VerticeNum] into &Vertice, return negative value on error
		VerticeData GetVerticeData(const unsigned int &VerticeNum) const;						// Returns vertice without error checking

		int GetEdgeData(const OBSTACLE_ID &EdgeID, EdgeData &EdgeDat) const;		// Stores edges[EdgeNum] into &Edge, return negative value on error
		EdgeData GetEdgeData(const OBSTACLE_ID &EdgeID) const;				// Returns edge without error checking

		int GetAdjoiningEdges(const OBSTACLE_ID &VerticeID, std::vector<OBSTACLE_ID> &AdjoiningEdgeIDs) const;		// Returns IDs of all connected edges
		int GetAdjoiningVerticesFromVertice(const OBSTACLE_ID &VerticeID, std::vector<OBSTACLE_ID> &AdjoiningVerticeIDs) const;	// Returns IDs of all connected vertices

		int GetAdjoiningVerticesFromEdge(const OBSTACLE_ID &EdgeID, std::array<OBSTACLE_ID, 2> &AdjoiningVertices) const;		// Returns IDs of all connected vertices

	private:

		std::vector<GraphEdge<EdgeData>>		_Edges;		// Edges of graph
		std::vector<GraphVertice<VerticeData>>	_Vertices;	// Vectors of graph

		const unsigned int NULL_POINTER = UINT_MAX;
};

#endif // GRAPHCLASS_H
