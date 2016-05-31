/* MapGenerator.h
 * 	Includes class MapGenerator
 */
#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

/* class MapGenerator
 * 	used to create a random rectangular map, given a seed, height and width
 */

#include "map.h"

// Enumerator with directions
enum DIRECTION
{
	UP=0,DOWN=1,LEFT=2,RIGHT=3
};

// Path data
struct MAPGEN_PATH
{
		POS_2D StartPos;
		unsigned int PathWidth;
		unsigned int PathLength;
		char PathDir;
};

class MapGenerator
{
	public:
		MapGenerator();
		MapGenerator(const MapGenerator &S);
		~MapGenerator();

		void SetupParameters(unsigned int Height, unsigned int Width, unsigned int MinPathWidth, unsigned int MaxPathWidth, unsigned int MinPathLength, unsigned int MaxPathLength, unsigned int VehicleRadius,unsigned int NumPaths);

		int CreateMap(int Seed, Map_BoolType &NewMap);

	private:

		MAPGEN_PATH CreateRandomPath(POS_2D StartPos, char Direction, unsigned int MinPathWidth);
		int DrawPathUntilIntersect(Map_BoolType &MapData, MAPGEN_PATH &NewPath, bool &IsConnected);
		void UpdateValidIntersectMap(Map_BoolType &ValidIntersectMap, const MAPGEN_PATH &NewPath);

		unsigned int _Height, _Width;						// Height and Width of created map
		unsigned int _MinPathWidth, _MaxPathWidth;		// thickness of paths
		unsigned int _MinPathLength, _MaxPathLength;		// Minimum and maximum path lengths
		unsigned int _VehicleRadius;						// Radius of vehicle
		unsigned int _NumPaths;					// Number of intersections

		POS_2D _MinValidArea, _MaxValidArea;				// Area where there should be valid positions for intersections
};

#endif
