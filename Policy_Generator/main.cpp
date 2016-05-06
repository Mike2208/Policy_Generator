#include "opencl_interface.h"

#include <stdio.h>
#include <string.h>

#define MAP_SIZE_X		1000
#define MAP_SIZE_Y		1000

#define NUM_MAPS		5

int main()
{
	OpenCLInterface testInterface;

	testInterface.Initialize();

	return 1;
}
