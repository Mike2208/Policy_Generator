//#include "opencl_interface.h"
#include "monte_carlo_option.h"
#include "occupancy_grid_map.h"

#include <stdio.h>
#include <string.h>

#define MAP_SIZE_X		1000
#define MAP_SIZE_Y		1000

#define NUM_MAPS		5

int main()
{
	OGM_MAP tMap;
	tMap.ResetMap(2,3, OGM_CELL_FREE);
	tMap.SetPixel(1,0,OGM_CELL_OCCUPIED/2);
	tMap.SetPixel(1,1,OGM_CELL_OCCUPIED/3);

	OccupancyGridMap testMap;
	testMap.SetData(tMap);

	MonteCarloOption mcTest;
	mcTest.PerformMonteCarlo(testMap, POS_2D(0,0), POS_2D(2,1));

	return 1;
}
