/* MapGenerator.cpp
 * 	contains functions for class MapGenerator
 */

#include "map_generator.h"
#include "map_standards.h"
//#include "MapGenerator_intesection.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using std::rand;
using std::vector;

struct PATH_RNDVAL
{
		MAPGEN_PATH PathData;		// Current path data
		int RandomLimit;	// Value that needs to be underscored for this path to be chosen
};

// Returns a random value between two borders
unsigned int GetRandomValue(unsigned int MinVal, unsigned int MaxVal)
{
	return rand()%(MaxVal-MinVal+1)+MinVal;
}

// Generates a random position in the rectangle created by MinPos and MaxPos
POS_2D GetRandomPosition(POS_2D MinPos, POS_2D MaxPos)
{
	POS_2D newPos;
	newPos.X = rand()%(MaxPos.X-MinPos.X+1) + MinPos.X;
	newPos.Y = rand()%(MaxPos.Y-MinPos.Y+1) + MinPos.Y;

	return newPos;
}

POS_2D GetNewPosition(POS_2D CurPos, char MovementDir, unsigned int MoveLength)
{
	POS_2D NewPos;
	switch(MovementDir)
	{
		case RIGHT:
			NewPos.X = CurPos.X + MoveLength;
			NewPos.Y = CurPos.Y;
		break;

		case LEFT:
			NewPos.X = CurPos.X - MoveLength;
			NewPos.Y = CurPos.Y;
		break;

		case UP:
			NewPos.Y = CurPos.Y + MoveLength;
			NewPos.X = CurPos.X;
		break;

		case DOWN:
			NewPos.Y = CurPos.Y - MoveLength;
			NewPos.X = CurPos.X;
		break;
	}

	return NewPos;
}

void GetPathArea(const MAPGEN_PATH &Path, POS_2D &TopLeft, POS_2D &BottomRight)
{
	// Calculate path to hollow out
	//	NOTE: A little bit of extra space is hollowed out at edges to ensure that all corners have the correct size
	switch(Path.PathDir)
	{
		case RIGHT:
			TopLeft.X = Path.StartPos.X - Path.PathWidth/2;
			TopLeft.Y = Path.StartPos.Y + Path.PathWidth/2;
			BottomRight.X = TopLeft.X + (Path.PathLength) + (Path.PathWidth-11);
			BottomRight.Y = TopLeft.Y - (Path.PathWidth-1);
		break;

		case LEFT:
			BottomRight.X = Path.StartPos.X + Path.PathWidth/2;
			BottomRight.Y = Path.StartPos.Y - Path.PathWidth/2;
			TopLeft.X = BottomRight.X - (Path.PathLength) - (Path.PathWidth-1);
			TopLeft.Y = BottomRight.Y + (Path.PathWidth-1);
		break;

		case UP:
			BottomRight.X = Path.StartPos.X + Path.PathWidth/2;
			BottomRight.Y = Path.StartPos.Y - Path.PathWidth/2;
			TopLeft.X = BottomRight.X - (Path.PathWidth);
			TopLeft.Y = BottomRight.Y + (Path.PathLength) + (Path.PathWidth-1);
		break;

		case DOWN:
			TopLeft.X = Path.StartPos.X - Path.PathWidth/2;
			TopLeft.Y = Path.StartPos.Y + Path.PathWidth/2;
			BottomRight.X = TopLeft.X + (Path.PathWidth-1);
			BottomRight.Y = TopLeft.Y - (Path.PathLength) - (Path.PathWidth-1);
		break;
	}
}

MapGenerator::MapGenerator()
{

}

MapGenerator::~MapGenerator()
{
}

// Will create random path data in the given constraints (StartPos must be at least MinPathWidth/2 from edes!!!)
MAPGEN_PATH MapGenerator::CreateRandomPath(POS_2D StartPos, char Direction, unsigned int MinPathWidth)
{
	MAPGEN_PATH newPath;
	newPath.StartPos = StartPos;
	newPath.PathDir = Direction;

	// Create a random path width
	newPath.PathWidth = GetRandomValue(MinPathWidth, this->_MaxPathWidth);

	// Create a random path length
	newPath.PathLength = GetRandomValue(this->_MinPathLength, this->_MaxPathLength);

	// Check that it is still within the area
	switch(Direction)
	{
		case UP:
			// check length
			if(StartPos.Y + newPath.PathLength > this->_Height)
				newPath.PathLength = this->_Height - StartPos.Y;

//			// Check width (on both edges)
//			if(StartPos.X + newPath.PathWidth/2 > this->width)
//				newPath.PathWidth = (this->width - StartPos.X)*2;
//			if(StartPos.X > newPath.PathWidth/2)
//				newPath.PathWidth = StartPos.X * 2;

		break;

		case DOWN:
			// check length
			if(StartPos.Y < newPath.PathLength)
				newPath.PathLength = StartPos.Y;

//			// Check width (on both edges)
//			if(StartPos.X + newPath.PathWidth/2 > this->width)
//				newPath.PathWidth = (this->width - StartPos.X)*2;
//			if(StartPos.X < newPath.PathWidth/2)
//				newPath.PathWidth = StartPos.X * 2;

		break;

		case RIGHT:
			// check length
			if(StartPos.X + newPath.PathLength > this->_Width)
				newPath.PathLength = this->_Width - StartPos.X;

//			// Check width (on both edges)
//			if(StartPos.Y + newPath.PathWidth/2 > this->height)
//				newPath.PathWidth = (this->height - StartPos.Y)*2;
//			if(StartPos.Y > newPath.PathWidth/2)
//				newPath.PathWidth = StartPos.Y * 2;

		break;

		case LEFT:
			// check length
			if(StartPos.X < newPath.PathLength )
				newPath.PathLength = StartPos.X;

//			// Check width (on both edges)
//			if(StartPos.Y + newPath.PathWidth/2 > this->height)
//				newPath.PathWidth = (this->height - StartPos.Y)*2;
//			if(StartPos.Y < newPath.PathWidth/2)
//				newPath.PathWidth = StartPos.Y * 2;

		break;
	};

	return newPath;
}


void MapGenerator::SetupParameters(unsigned int Height, unsigned int Width, unsigned int MinPathWidth, unsigned int MaxPathWidth, unsigned int MinPathLength, unsigned int MaxPathLength, unsigned int VehicleRadius,unsigned int NumPaths)
{
	// Add parameters
	this->_Height = Height;
	this->_Width = Width;

	this->_MinPathWidth = MinPathWidth;
	this->_MaxPathWidth = MaxPathWidth;

	this->_MinPathLength = MinPathLength;
	this->_MaxPathLength = MaxPathLength;

	this->_VehicleRadius = VehicleRadius;

	this->_NumPaths = NumPaths;
}

/* CreateMap()
 * 	This function creates a new random map. It does this in the following way:
 * 	 1. 	It will create a new, completely full map
 * 	 2. 	The top of the map is hollowed out ( this ensures that there will always be a path for the robot )
 * 	 3.	It will select a random point as a new intersection
 * 	 4. 	If this intersection is too close to an existing one (less than VehicleRadius apart from each other), generate a new random position
 * 	 5. 	A square area with width and height VehicleRadius is hollowed out around this intersection
 * 	 6. 	This intersection will get 1,2,3, or 4 different paths coming from it
 * 	 7. 	It is randomly decided in which direction the path will go (either North, South, East, or West, but not diagonally)
 * 	 8. 	It is randomly decided how thick and long the line is,  somewhere between Min- and MaxPathWidth, and Min- and MaxPathLength
 * 	 9. 	If this path intersects with other ones, a new intersection is created at these points, and the NumIntersections counter is incremented
 * 	 10. 	If the intersection is not connected with any other line, 
 * 	 11. 	Steps 2-9 are repeated until the desired number of intersections is reached
 */
int MapGenerator::CreateMap(int Seed, Map_BoolType &NewMap)
{
	// Seed random number generator
	std::srand(Seed);

	// Create storage for Paths
	vector<PATH_RNDVAL> storedPaths;

	int totalRndValues = 0;

	// Reset Map to full, the paths will be hollowed out later
	NewMap.ResetMap(this->_Height, this->_Width, OCCUPANCYGRID_DISCRETE_FULL);

	unsigned int curNumPaths = 0;

	POS_2D minPos, maxPos;
	minPos.X = this->_MaxPathWidth/2+1;
	minPos.Y = this->_MaxPathWidth/2+1;

	maxPos.X = this->_Width - minPos.X;
	maxPos.Y = this->_Height - minPos.Y;

	// Create first path

	PATH_RNDVAL curPath;
	POS_2D topLeft, bottomRight;

	// Select random point on map
	curPath.PathData.StartPos = GetRandomPosition(minPos, maxPos);

	// Start at center of map
	curPath.PathData.StartPos.X = this->_Width/2;
	curPath.PathData.StartPos.Y = this->_Height/2;

	// Select random direction
	curPath.PathData.PathDir = rand()%4;

	// Create random path with at least the size of the vehicle as width
	curPath.PathData = CreateRandomPath(curPath.PathData.StartPos, curPath.PathData.PathDir, std::max(this->_VehicleRadius, this->_MinPathWidth));

	// Create path in center, from left to right
	//curPath.PathData.StartPos.X = std::max(this->vehicleRadius, this->minPathWidth)/2;
	//curPath.PathData.StartPos.Y = this->height/2;

	//curPath.PathData.PathDir = RIGHT;
	//curPath.PathData.PathLength = this->width - (curPath.PathData.StartPos.X*2+1);
	//curPath.PathData.PathWidth = curPath.PathData.StartPos.X*2;

	// Create path to hollow out
	//	NOTE: A little bit of extra space is hollowed out at edges to ensure that all corners have the correct size
	GetPathArea(curPath.PathData, topLeft, bottomRight);
	NewMap.SetArea(topLeft, bottomRight, OCCUPANCYGRID_DISCRETE_EMPTY);

	// Update Rand Limit with length of current path
	curPath.RandomLimit = totalRndValues + curPath.PathData.PathLength;
	totalRndValues += curPath.PathData.PathLength;

	// Add new path to vector
	storedPaths.resize(1);
	storedPaths[0] = curPath;

	// Update NumPaths
	curNumPaths++;

	// Go through rest of paths that should be created
	while(curNumPaths < this->_NumPaths)
	{
		// Choose a random path that has valid start and end points
		do
		{

			// Select random point on given paths
			int randVal = rand()%totalRndValues;
			unsigned int startPathID=0;				// position in vector of startPath
			MAPGEN_PATH *startPath;
			while(startPathID < storedPaths.size())
			{
				if(storedPaths[startPathID].RandomLimit > randVal)
				{
					// This value is too high, select lower one
					break;
				}

				startPathID++;
			}
			startPath = &(storedPaths[startPathID].PathData);
			curPath.PathData.StartPos = GetNewPosition(startPath->StartPos, startPath->PathDir, storedPaths[startPathID].RandomLimit - randVal - 1);

			// Select random direction, depending on direction of start Path
			// New path should be perpendicular
			curPath.PathData.PathDir = rand()%2;
			if(startPath->PathDir <= 1)
				curPath.PathData.PathDir += 2;			// switches from LEFT/RIGHT to UP/DOWN (seen DIRECTION enumerator)

			// Create random path
			curPath.PathData = CreateRandomPath(curPath.PathData.StartPos, curPath.PathData.PathDir, this->_MinPathWidth);

			// Create path to hollow out
			//	NOTE: A little bit of extra space is hollowed out at edges to ensure that all corners have the correct size
			GetPathArea(curPath.PathData, topLeft, bottomRight);
		}
		// Continue until all points are within valid range
		while(topLeft.X < 0 || topLeft.X >= this->_Width || topLeft.Y < 0 || topLeft.Y >= this->_Height ||
				bottomRight.X < 0 || bottomRight.X >= this->_Width || bottomRight.Y < 0 || bottomRight.Y >= this->_Height);


		// Hollow out path
		NewMap.SetArea(topLeft, bottomRight, OCCUPANCYGRID_DISCRETE_EMPTY);

		// Check if path is wide enough for vehicle, if yes, then add it to stored paths
		if(curPath.PathData.PathWidth >= this->_VehicleRadius)
		{
			// Update Rand Limit with length of current path
			curPath.RandomLimit = totalRndValues + curPath.PathData.PathLength;
			totalRndValues += curPath.PathData.PathLength;

			// Add new path to vector
			storedPaths.push_back(curPath);
		}

		// Update NumPaths
		curNumPaths++;
	}

	// Copy data to requester
	//NewMap = curMap;

	return 1;
}

//void MapGenerator::UpdateValidIntersectMap(Map &ValidIntersectMap, const PATH &NewPath)
//{
//	// Fill out area around path
//	POS_2D areaTopLeft, areaBottomRight;

//	// Check direction
//	switch(NewPath.PathDir)
//	{
//		case RIGHT:
//			areaTopLeft.X = NewPath.StartPos.X;
//			areaTopLeft.Y = NewPath.StartPos.Y + NewPath.PathWidth/2 + this->maxPathLength - 1;

//			areaBottomRight.X = NewPath.StartPos.X + NewPath.PathLength;
//			areaBottomRight.Y = NewPath.StartPos.Y - NewPath.PathWidth/2 - this->maxPathLength + 1;

//			if(areaTopLeft.Y > this->height)
//				areaTopLeft.Y = this->height;
//			if(areaBottomRight.Y > this->height)
//				areaBottomRight.Y = 0;
//		break;

//		case LEFT:
//			areaBottomRight.X = NewPath.StartPos.X;
//			areaBottomRight.Y = NewPath.StartPos.Y - NewPath.PathWidth/2 - this->maxPathLength - 1;

//			areaTopLeft.X = NewPath.StartPos.X + NewPath.PathLength;
//			areaTopLeft.Y = NewPath.StartPos.Y + NewPath.PathWidth/2 + this->maxPathLength + 1;

//			if(areaTopLeft.Y > this->height)
//				areaTopLeft.Y = this->height;
//			if(areaBottomRight.Y > this->height)
//				areaBottomRight.Y = 0;
//		break;

//		case DOWN:
//			areaTopLeft.Y = NewPath.StartPos.Y;
//			areaTopLeft.X = NewPath.StartPos.X - NewPath.PathWidth/2 - this->maxPathLength + 1;

//			areaBottomRight.Y = NewPath.StartPos.Y + NewPath.PathLength;
//			areaBottomRight.X = NewPath.StartPos.X + NewPath.PathWidth/2 + this->maxPathLength - 1;

//			if(areaTopLeft.X > this->width)
//				areaTopLeft.X = this->width;
//			if(areaBottomRight.X > this->width)
//				areaBottomRight.X = 0;
//		break;

//		case UP:
//			areaBottomRight.Y = NewPath.StartPos.Y;
//			areaBottomRight.X = NewPath.StartPos.X + NewPath.PathWidth/2 + this->maxPathLength - 1;

//			areaTopLeft.Y = NewPath.StartPos.Y + NewPath.PathLength;
//			areaTopLeft.X = NewPath.StartPos.X - NewPath.PathWidth/2 - this->maxPathLength + 1;

//			if(areaTopLeft.X > this->width)
//				areaTopLeft.X = this->width;
//			if(areaBottomRight.X > this->width)
//				areaBottomRight.X = 0;
//		break;
//	}

//	// Check if the valid area was expanded
//	if(this->minValidArea.X > areaTopLeft.X)
//		this->minValidArea.X = areaTopLeft.X;

//	if(this->minValidArea.Y < areaTopLeft.Y)
//		this->minValidArea.Y = areaTopLeft.Y;


//	if(this->maxValidArea.X < areaBottomRight.X)
//		this->maxValidArea.X = areaBottomRight.X;

//	if(this->maxValidArea.Y > areaBottomRight.Y)
//		this->maxValidArea.Y = areaBottomRight.Y;


//	// Mark area as valid
//	POS_2D curPos;
//	for (curPos.X = areaTopLeft.X; curPos.X <= areaBottomRight.X; curPos.X++)
//	{
//		for(curPos.Y = areaTopLeft.Y; curPos.Y >= areaBottomRight.Y; curPos.Y--)
//		{
//			ValidIntersectMap.SetPixel(curPos, 1);
//		}
//	}

//	return;
//}

//unsigned int GetLineLength(Map &curMap, POS_2D_TYPE PosX, POS_2D_TYPE PosY, DIRECTION Direction)
//{
//	unsigned int lineLength = 0;

//	while(curMap.GetPixel(PosX, PosY) == OCCUPANCYGRID_DISCRETE_EMPTY)
//	{
//		lineLength++;

//		// Move along given direction
//		switch (Direction)
//		{
//			case RIGHT:
//				curX++;
//			break;
//			case LEFT:
//				curX--;
//			break;
//			case UP:
//				curY++;
//			break;
//			case DOWN:
//				curY--;
//			break;
//		}
//	}

//	return lineLength;
//}
