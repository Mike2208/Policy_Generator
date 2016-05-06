#include "opencl_interface.h"

OpenCLInterface::OpenCLInterface()
{

}

OpenCLInterface::~OpenCLInterface()
{
	this->Close();
}

int OpenCLInterface::Initialize()
{
	cl::Platform::get(&this->_Platforms);

	this->_Platforms[0].getDevices(CL_DEVICE_TYPE_GPU, &this->_Devices);

	return 1;
}

int OpenCLInterface::Close()
{
	return -1;
}
