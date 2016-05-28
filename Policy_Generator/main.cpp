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
	tMap.ResetMap(1,3, 0);
	tMap.SetPixel(1,0,OGM_CELL_OCCUPIED);

	OccupancyGridMap testMap;
	testMap.SetData(tMap);

	MonteCarloOption mcTest;
	mcTest.PerformMonteCarlo(testMap, POS_2D(0,0), POS_2D(2,0));

	return 1;
}
