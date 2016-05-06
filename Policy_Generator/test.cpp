#include "map_generator.h"
#include "map_path_finder.h"
#include "map_comparator.h"
#include "map_height_map.h"
#include "obstacle_path_finder.h"
#include <stdio.h>
#include <string.h>

#define MAP_SIZE_X		1000
#define MAP_SIZE_Y		1000

#define NUM_MAPS		5


int TestMapGen()
{
	MapGenerator testGen;
	Map<OCCUPANCYGRID_DISCRETE_TYPE> newMap[NUM_MAPS];

	char fileName[100] = "/tmp/t0estFile.pbm";
	testGen.SetupParameters(MAP_SIZE_X, MAP_SIZE_Y, 10, 15, 200, 500, 5, 700);

	for(int i=0; i<NUM_MAPS; i++)
	{
		testGen.CreateMap(i+342, newMap[i]);

		fileName[6] = i+48;
		newMap[i].PrintMapToFile(fileName, 1);
	}

	MapComparator testComp;
	Map_BoolType compMap;
	testComp.CompareMaps(newMap[0], newMap[1], compMap);

	compMap.PrintMapToFile("/tmp/compMap.pgm", 1);

	return 1;
}

int TestObstaclePathFinder()
{
	Map_BoolType obstacleMap;
	obstacleMap.ResetMap(500, 500, OCCUPANCYGRID_DISCRETE_EMPTY);

	std::vector<POS_2D> obstaclePoses;
	obstaclePoses.push_back(POS_2D(500/4, 500/2));
	obstaclePoses.push_back(POS_2D(500/4*3, 500/2));

	obstacleMap.SetPixel(obstaclePoses[0], OCCUPANCYGRID_DISCRETE_FULL);
	obstacleMap.SetPixel(obstaclePoses[1], OCCUPANCYGRID_DISCRETE_FULL);

	ObstacleConnections connections;

	ObstaclePathFinder::FindAdjoiningObstacles(obstacleMap, obstaclePoses, POS_2D(500/2, 500/4), POS_2D(500/2, 500/4*3), connections);

	return 1;
}
