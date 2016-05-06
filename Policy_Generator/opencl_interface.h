#ifndef OPENCL_INTERFACE_H
#define OPENCL_INTERFACE_H

/*	class OpenCLInterface
 *		interface to OpenCL
 *		used to access graphic card
 */

#include <CL/cl.hpp>

class OpenCLInterface
{
	public:
		OpenCLInterface();
		~OpenCLInterface();

		int Initialize();

		int Close();

	private:

		std::vector<cl::Platform>	_Platforms;
		std::vector<cl::Device>		_Devices;
};

#endif // OPENCL_INTERFACE_H
