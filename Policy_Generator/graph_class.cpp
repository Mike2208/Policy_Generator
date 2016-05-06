#ifndef GRAPH_CLASS_CPP
#define GRAPH_CLASS_CPP

#include "graph_class.h"

template<class VerticeData, class EdgeData>
GraphClass<VerticeData, EdgeData>::GraphClass()
{

}

template<class VerticeData, class EdgeData>
void GraphClass<VerticeData, EdgeData>::Reset()
{
	this->_Edges.clear();
	this->_Vertices.clear();
}

template<class VerticeData, class EdgeData>
void GraphClass<VerticeData, EdgeData>::SetStartVertice(const GraphVertice<VerticeData> &StartVertice)
{
	// Clear all data
	this->Reset();

	// Add new Vertice
	this->_Vertices.push_back(StartVertice);

	// Clear edges of vertice if present
	this->_Vertices[0].ClearEdgePointers();
}

//template<class VerticeData, class EdgeData>
//int GraphClass<VerticeData, EdgeData>::AddEdge(const GraphEdge<EdgeData> &NewEdge, const unsigned int &VerticeID, const bool &ConnectAtStart)
//{
//	// Check that vertice is valid
//	if(VerticeID >= this->vertices.size())
//		return -1;

//	// Add edge to vector
//	const unsigned int newEdgeNum = this->edges.size();
//	this->edges.push_back(NewEdge);

//	// Connect new edge to given Vertice
//	this->vertices[VerticeID].AddEdgePointer(newEdgeNum);

//	// Connect vertice to edge
//	if(ConnectAtStart)
//	{
//		this->edges[newEdgeNum].SetVertice1(VerticeID);
//		this->edges[newEdgeNum].SetVertice2(GraphClass::NULL_POINTER);
//	}
//	else
//	{
//		this->edges[newEdgeNum].SetVertice1(GraphClass::NULL_POINTER);
//		this->edges[newEdgeNum].SetVertice2(VerticeID);
//	}

//	return 1;
//}

template<class VerticeData, class EdgeData>
int GraphClass<VerticeData, EdgeData>::AddNewVertice(const VerticeData &NewVerticeData, const EdgeData &NewEdgeData, const unsigned int &ConnectVerticeID, const bool &NewEndVertice)
{
	// Check that VericeID is valid
	if(ConnectVerticeID >= this->_Vertices.size())
		return -1;

	// Create new Vertice with the given data
	const unsigned int newVerticeID = this->_Vertices.size();
	this->_Vertices.resize(newVerticeID+1);
	this->_Vertices[newVerticeID].GraphVertice();		// Call constructor to prevent errors (not sure if necessary)
	this->_Vertices[newVerticeID].SetVerticeData(NewVerticeData);

	// Connect both vertices by creating a new edge
	const unsigned int newEdgeID = this->_Edges.size();
	this->_Edges.resize(newEdgeID+1);
	this->_Edges[newEdgeID].GraphEdge();					// Call constructor to prevent errors (not sure if necessary)
	this->_Edges[newEdgeID].SetEdgeData(NewEdgeData);

	const GraphVertice<VerticeData> &oldVerticeRef = this->_Vertices[ConnectVerticeID];
	const GraphVertice<VerticeData> &newVerticeRef = this->_Vertices[newVerticeID];
	const GraphEdge<EdgeData> &newEdgeRef = this->_Edges[newEdgeID];

	// Connect vertices to edge
	newVerticeRef.AddEdgeID(newEdgeID);
	oldVerticeRef.AddEdgeID(newEdgeID);

	if(NewEndVertice)
	{
		// Connect new vertice at end of edge, old vertice at start
		newEdgeRef.SetVertice1(ConnectVerticeID);
		newEdgeRef.SetVertice2(newVerticeID);
	}
	else
	{
		// Connect old vertice at end of edge, new vertice at start
		newEdgeRef.SetVertice1(newVerticeID);
		newEdgeRef.SetVertice2(oldVerticeRef);
	}
}

template<class VerticeData, class EdgeData>
int GraphClass<VerticeData, EdgeData>::ConnectVertices(const unsigned int &StartVerticeID, const unsigned int &EndVerticeID, const EdgeData &NewEdgeData)
{
	// Check that both vertices exist
	if(StartVerticeID >= this->_Vertices.size() || EndVerticeID >= this->_Vertices.size())
		return -1;

	const GraphVertice<VerticeData> &startVerticeRef = this->_Vertices[StartVerticeID];
	const GraphVertice<VerticeData> &endVerticeRef = this->_Vertices[EndVerticeID];

	// Check that both vertices aren't connected yet
	for(unsigned int i=0; i < startVerticeRef.GetNumEdgePointers(); i++)
	{
		const GraphEdge<EdgeData> &edgeConnectRef = this->_Edges[startVerticeRef.GetEdgePointer(i)];

		// Return error if both vertices are already connected
		if(edgeConnectRef.GetVertice2() == EndVerticeID || edgeConnectRef.GetVertice1() == EndVerticeID)
			return -2;
	}

	// Connect both vertices by creating a new edge
	const unsigned int newEdgeID = this->_Edges.size();
	this->_Edges.resize(newEdgeID+1);
	this->_Edges[newEdgeID].GraphEdge();					// Call constructor to prevent errors (not sure if necessary)
	this->_Edges[newEdgeID].SetEdgeData(NewEdgeData);

	const GraphEdge<EdgeData> &newEdgeRef = this->_Edges[newEdgeID];

	// Connect vertices to edge
	startVerticeRef.AddEdgeID(newEdgeID);
	endVerticeRef.AddEdgeID(newEdgeID);

	// Connect new vertice at end of edge, old vertice at start
	newEdgeRef.SetVertice1(StartVerticeID);
	newEdgeRef.SetVertice2(EndVerticeID);

	return 1;
}

template<class VerticeData, class EdgeData>
int GraphClass<VerticeData, EdgeData>::GetVerticeData(const unsigned int &VerticeID, VerticeData &VerticeDat) const
{
	// Check that vertice exists
	if(VerticeID >= this->_Vertices.size())
		return -1;

	// Copy vertice data
	VerticeDat = this->_Vertices[VerticeID].GetVerticeData();

	return 1;
}

template<class VerticeData, class EdgeData>
VerticeData GraphClass<VerticeData, EdgeData>::GetVerticeData(const unsigned int &VerticeID) const
{
	// Copy vertice data
	return this->_Vertices[VerticeID].GetVerticeData();
}

template<class VerticeData, class EdgeData>
int GraphClass<VerticeData, EdgeData>::GetEdgeData(const unsigned int &EdgeID, EdgeData &EdgeDat) const
{
	// Check that edge exists
	if(EdgeID >= this->_Edges.size())
		return -1;

	// Copy edge data
	EdgeDat = this->_Edges[EdgeID].GetEdgeData();

	return 1;

}

template<class VerticeData, class EdgeData>
EdgeData GraphClass<VerticeData, EdgeData>::GetEdgeData(const unsigned int &EdgeID) const
{
	// Copy edge data
	return this->_Edges[EdgeID].GetEdgeData();
}

template<class VerticeData, class EdgeData>
int GraphClass<VerticeData, EdgeData>::GetAdjoiningEdges(const unsigned int &VerticeID, std::vector<unsigned int> &AdjoiningEdgeIDs) const
{
	// Check that vertice exists
	if(VerticeID >= this->_Vertices.size())
		return -1;

	const GraphVertice<VerticeData> &oldVerticeRef;

	// Get number of connected edges
	const unsigned int numAdjoiningEdges = oldVerticeRef.GetNumEdgePointers();
	AdjoiningEdgeIDs.resize(numAdjoiningEdges);

	// Get all Ids
	for(unsigned int i=0; i<numAdjoiningEdges; i++)
	{
		AdjoiningEdgeIDs[i] = oldVerticeRef.GetEdgeID(i);
	}

	return 1;
}


template<class VerticeData, class EdgeData>
int GraphClass<VerticeData, EdgeData>::GetAdjoiningVerticesFromVertice(const unsigned int &VerticeID, std::vector<unsigned int> &AdjoiningVerticeIDs) const
{
	// Check that vertice exists
	if(VerticeID >= this->_Vertices.size())
		return -1;

	const GraphVertice<VerticeData> &oldVerticeRef;

	// Get number of connected edges (same as number of edges)
	const unsigned int numAdjoiningVertices = oldVerticeRef.GetNumEdgePointers();
	AdjoiningVerticeIDs.resize(numAdjoiningVertices);

	// Get all IDs
	for(unsigned int i=0; i<numAdjoiningVertices; i++)
	{
		const GraphEdge<EdgeData> edgeRef = oldVerticeRef.GetEdgeID(i);

		// Check which vertice is the connected one and add it to vector
		const unsigned int vertice1ID = edgeRef.GetVertice1();
		if(vertice1ID == VerticeID)
			AdjoiningVerticeIDs[i] = edgeRef.GetVertice2();
		else
			AdjoiningVerticeIDs[i] = vertice1ID;
	}

	return 1;
}

template<class VerticeData, class EdgeData>
int GraphClass<VerticeData, EdgeData>::GetAdjoiningVerticesFromEdge(const unsigned int &EdgeID, std::array<unsigned int, 2> &AdjoiningVertices) const
{
	// Check that edge exists
	if(EdgeID >= this->_Edges.size())
		return -1;

	const GraphEdge<EdgeData> &edgeRef;

	// Save ID of adjoining edges
	AdjoiningVertices[0] = edgeRef.GetVertice1();
	AdjoiningVertices[1] = edgeRef.GetVertice2();

	return 1;
}

#endif
