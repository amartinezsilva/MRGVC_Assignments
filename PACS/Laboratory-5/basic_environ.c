////////////////////////////////////////////////////////////////////
//File: basic_environ.c
//
//Description: base file for environment exercises with openCL
//
// 
////////////////////////////////////////////////////////////////////

#define cimg_use_jpeg
#include "CImg/CImg.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifdef __APPLE__
  #include <OpenCL/opencl.h>
#else
  #include <CL/cl.h>
#endif

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <iomanip>

using namespace cimg_library;
  
// check error, in such a case, it exits

void cl_error(cl_int code, const char *string){
	if (code != CL_SUCCESS){
		printf("%d - %s\n", code, string);
	    exit(-1);
	}
}
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  clock_t timer;
	timer = clock();

  cl_event kernel_time;
  cl_event kernel_write_bandwidth;
  cl_event kernel_read_bandwidth;

  int err;                            	// error code returned from api calls
  size_t t_buf = 50;			// size of str_buffer
  char str_buffer[t_buf];		// auxiliary buffer	

  printf("Memory Footprint of the auxiliar buffer: %zu bytes\n", t_buf * sizeof(char));

  size_t e_buf;				// effective size of str_buffer in use
	    
  //size_t global_size;                      	// global domain size for our calculation
  size_t local_size;                       	// local domain size for our calculation

  const cl_uint num_platforms_ids = 10;				// max of allocatable platforms
  cl_platform_id platforms_ids[num_platforms_ids];		// array of platforms
  cl_uint n_platforms;						// effective number of platforms in use
  const cl_uint num_devices_ids = 10;				// max of allocatable devices
  cl_device_id devices_ids[num_platforms_ids][num_devices_ids];	// array of devices
  cl_uint n_devices[num_platforms_ids];				// effective number of devices in use for each platform
	
  cl_device_id device_id;             				// compute device id 
  cl_context context;                 				// compute context
  cl_command_queue command_queue;     				// compute command queue

  // 1. Scan the available platforms:
  err = clGetPlatformIDs (num_platforms_ids, platforms_ids, &n_platforms);
  cl_error(err, "Error: Failed to Scan for Platforms IDs");
  printf("Number of available platforms: %d\n\n", n_platforms);

  for (int i = 0; i < n_platforms; i++ ){
    size_t platformNameSize;
    err= clGetPlatformInfo(platforms_ids[i], CL_PLATFORM_NAME, sizeof(str_buffer), &str_buffer, NULL);
    cl_error (err, "Error: Failed to get info of the platform\n");
    printf( "\t[%d]-Platform Name: %s\n", i, str_buffer);
  }
  printf("\n");
	
  // 2. Scan for devices in each platform
  for (int i = 0; i < n_platforms; i++ ){
    err = clGetDeviceIDs( platforms_ids[i], CL_DEVICE_TYPE_ALL , num_devices_ids, devices_ids[i], &(n_devices[i]));
    cl_error(err, "Error: Failed to Scan for Devices IDs");
    printf("\t[%d]-Platform. Number of available devices: %d\n", i, n_devices[i]);

    for(int j = 0; j < n_devices[i]; j++){
      err = clGetDeviceInfo(devices_ids[i][j], CL_DEVICE_NAME, sizeof(str_buffer), &str_buffer, NULL);
      cl_error(err, "clGetDeviceInfo: Getting device name");
      printf("\t\t [%d]-Platform [%d]-Device CL_DEVICE_NAME: %s\n", i, j,str_buffer);

      cl_uint max_compute_units_available;
      err = clGetDeviceInfo(devices_ids[i][j], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(max_compute_units_available), &max_compute_units_available, NULL);
      cl_error(err, "clGetDeviceInfo: Getting device max compute units available");
      printf("\t\t [%d]-Platform [%d]-Device CL_DEVICE_MAX_COMPUTE_UNITS: %d\n\n", i, j, max_compute_units_available);

      // ***Task***: print on the screen the cache size, global mem size, local memsize, max work group size, profiling timer resolution and ... of each device

      // cache size
      cl_ulong cache_size;
      err = clGetDeviceInfo(devices_ids[i][j], CL_DEVICE_GLOBAL_MEM_CACHE_SIZE, sizeof(cache_size), &cache_size, NULL);
      cl_error(err, "clGetDeviceInfo: Getting device cache size");
      printf("\t\t [%d]-Platform [%d]-Device CL_DEVICE_GLOBAL_MEM_CACHE_SIZE: %lu\n", i, 0, cache_size);

      // global mem size
      cl_ulong global_mem_size;
      err = clGetDeviceInfo(devices_ids[i][j], CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(global_mem_size), &global_mem_size, NULL);
      cl_error(err, "clGetDeviceInfo: Getting device global mem size");
      printf("\t\t [%d]-Platform [%d]-Device CL_DEVICE_GLOBAL_MEM_SIZE: %lu\n", i, 0, global_mem_size);

      // local mem size
      cl_ulong local_mem_size;
      err = clGetDeviceInfo(devices_ids[i][j], CL_DEVICE_LOCAL_MEM_SIZE, sizeof(local_mem_size), &local_mem_size, NULL);
      cl_error(err, "clGetDeviceInfo: Getting device local mem size");
      printf("\t\t [%d]-Platform [%d]-Device CL_DEVICE_LOCAL_MEM_SIZE: %lu\n", i, 0, local_mem_size);

      // max work group size
      size_t max_work_group_size;
      err = clGetDeviceInfo(devices_ids[i][j], CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(max_work_group_size), &max_work_group_size, NULL);
      cl_error(err, "clGetDeviceInfo: Getting device max work group size");
      printf("\t\t [%d]-Platform [%d]-Device CL_DEVICE_MAX_WORK_GROUP_SIZE: %d\n", i, 0, max_work_group_size);

      size_t max_work_item_dimensions;
      err = clGetDeviceInfo(devices_ids[i][j], CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(max_work_item_dimensions), &max_work_item_dimensions, NULL);
      cl_error(err, "clGetDeviceInfo: Getting device max work item dimensions");
      printf("\t\t [%d]-Platform [%d]-Device CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS: %d\n", i, 0, max_work_item_dimensions);

      // profiling timer resolution
      size_t profiling_timer_resolution;
      err = clGetDeviceInfo(devices_ids[i][j], CL_DEVICE_PROFILING_TIMER_RESOLUTION, sizeof(profiling_timer_resolution), &profiling_timer_resolution, NULL);
      cl_error(err, "clGetDeviceInfo: Getting device profiling timer resolution");
      printf("\t\t [%d]-Platform [%d]-Device CL_DEVICE_PROFILING_TIMER_RESOLUTION: %d\n", i, 0, profiling_timer_resolution);
    }
  }	


  // 3. Create a context, with a device
  cl_context_properties properties[] = { CL_CONTEXT_PLATFORM, (cl_context_properties)platforms_ids[0], 0};
  context = clCreateContext(properties, n_devices[0],&(devices_ids[0][0]), NULL, NULL, &err);
  cl_error(err, "Failed to create a compute context\n");

  // 4. Create a command queue
  cl_command_queue_properties proprt[] = { CL_QUEUE_PROPERTIES, CL_QUEUE_PROFILING_ENABLE, 0 };
  command_queue = clCreateCommandQueueWithProperties( context, devices_ids[0][0], proprt, &err);
  cl_error(err, "Failed to create a command queue\n");

  // 2. Load source code of kernel

  // Calculate size of the file
  FILE *fileHandler = fopen("kernel_rotation.cl", "r");
  fseek(fileHandler, 0, SEEK_END);
  size_t fileSize = ftell(fileHandler);
  rewind(fileHandler);

  printf("Memory Footprint of the file buffer: %d bytes\n", fileSize);

  // read kernel source into buffer
  char * sourceCode = (char*) malloc(fileSize + 1);
  sourceCode[fileSize] = '\0';
  fread(sourceCode, sizeof(char), fileSize, fileHandler);
  fclose(fileHandler);

  // create program from buffer
  cl_program program = clCreateProgramWithSource(context, 1, (const char**)&sourceCode, &fileSize, &err);
  cl_error(err, "Failed to create program with source\n");
  free(sourceCode);


  // 3. Build the executable and check errors

  err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
  if (err != CL_SUCCESS){
    size_t len;
    char buffer[2048];

    printf("Error: Some error at building process.\n");
    clGetProgramBuildInfo(program, device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
    printf("%s\n", buffer);
    exit(-1);
  }

  // 4. Create a compute kernel with the program we want to run
  cl_kernel kernel = clCreateKernel(program, "image_rotation", &err);
  cl_error(err, "Failed to create kernel from the program\n");

  // 5. Create and initialize input and output arrays at host memory
  CImg<unsigned char> image("image.jpg");  // Load image file "image.jpg" at object img
  // image size
  printf("Image size: %d\n", image.size());
  int width = image.width();
  int height = image.height();
  int spectrum = image.spectrum();

  printf("Image width: %d\n", width);
  printf("Image heigth: %d\n", height);
  printf("Image spectrum: %d\n", spectrum);

  size_t size = image.size();

  // 6. Create OpenCL buffer visible to the OpenCl runtime
  cl_ulong total_memory_allocated = 0;

  cl_mem in_device_object  = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(unsigned char)*size, NULL, &err);
  cl_error(err, "Failed to create memory buffer at device\n");
  cl_ulong in_device_mem_size;
  clGetMemObjectInfo(in_device_object, CL_MEM_SIZE, sizeof(cl_ulong), &in_device_mem_size, NULL);
  total_memory_allocated += in_device_mem_size;

  cl_mem out_device_object = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(unsigned char)*size, NULL, &err);
  cl_error(err, "Failed to create memory buffer at device\n");
  cl_ulong out_device_mem_size;
  clGetMemObjectInfo(out_device_object, CL_MEM_SIZE, sizeof(cl_ulong), &out_device_mem_size, NULL);
  total_memory_allocated += out_device_mem_size;

  printf("Memory Footprint of the Read/Write buffers: %lu bytes\n", total_memory_allocated);


  // 7. Write data into the memory object
  err = clEnqueueWriteBuffer(command_queue, in_device_object, CL_TRUE, 0, sizeof(unsigned char)*size,
                             image.data(), 0, NULL, &kernel_write_bandwidth);
  cl_error(err, "Failed to enqueue a write command\n");

  clWaitForEvents(1, &kernel_write_bandwidth);

  cl_ulong kernel_write_bandwidth_time_start, kernel_write_bandwidth_time_end;
  clGetEventProfilingInfo(kernel_write_bandwidth, CL_PROFILING_COMMAND_START, sizeof(cl_ulong), &kernel_write_bandwidth_time_start, NULL);
  clGetEventProfilingInfo(kernel_write_bandwidth, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &kernel_write_bandwidth_time_end, NULL);

  double kernel_write_bandwidth_time = (double)(kernel_write_bandwidth_time_end - kernel_write_bandwidth_time_start); 
  printf("OpenCl write buffer time is: %0.3f milliseconds \n",kernel_write_bandwidth_time / 1000000.0); // converted to miliseconds

  // 8. Set the arguments to the kernel
  err = clSetKernelArg(kernel, 0, sizeof(cl_mem), &in_device_object);
  cl_error(err, "Failed to set argument 0\n");
  err = clSetKernelArg(kernel, 1, sizeof(cl_mem), &out_device_object);
  err = clSetKernelArg(kernel, 2, sizeof(int), &width);
  cl_error(err, "Failed to set argument 1\n");
  err = clSetKernelArg(kernel, 3, sizeof(int), &height);
  cl_error(err, "Failed to set argument 2\n");
  float angle = 1.570796; //rotate 90 degrees
  err = clSetKernelArg(kernel, 4, sizeof(float), &angle);
  cl_error(err, "Failed to set argument 3\n");

  // 9. Launch Kernel

  const size_t global_size[3] = {image.width() , image.height(), image.spectrum()};

  err = clEnqueueNDRangeKernel(command_queue, kernel, 3, NULL, global_size, NULL, 0, NULL, &kernel_time);
  cl_error(err, "Failed to launch kernel to the device\n");

  // After the kernel execution
  clWaitForEvents(1, &kernel_time);

  cl_ulong kernel_time_start, kernel_time_end;
  clGetEventProfilingInfo(kernel_time, CL_PROFILING_COMMAND_START, sizeof(cl_ulong), &kernel_time_start, NULL);
  clGetEventProfilingInfo(kernel_time, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &kernel_time_end, NULL);

  double kernel_execution_time = (double)(kernel_time_end - kernel_time_start); 
  printf("OpenCl Kernel execution time is: %0.3f milliseconds \n",kernel_execution_time / 1000000.0); // converted to miliseconds


  // 10. Read data form device memory back to host memory
  CImg<unsigned char> image_out(image.width(), image.height(), 1, 3);
  err = clEnqueueReadBuffer(command_queue, out_device_object, CL_TRUE, 0,sizeof(unsigned char)*size, 
                            image_out.data(), 0, NULL, &kernel_read_bandwidth);
  cl_error(err, "Failed to enqueue a read command\n");

  clWaitForEvents(1, &kernel_read_bandwidth);

  cl_ulong kernel_read_bandwidth_time_start, kernel_read_bandwidth_time_end;
  clGetEventProfilingInfo(kernel_read_bandwidth, CL_PROFILING_COMMAND_START, sizeof(cl_ulong), &kernel_read_bandwidth_time_start, NULL);
  clGetEventProfilingInfo(kernel_read_bandwidth, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &kernel_read_bandwidth_time_end, NULL);

  double kernel_read_bandwidth_time = (double)(kernel_read_bandwidth_time_end - kernel_read_bandwidth_time_start); 
  printf("OpenCl read buffer time is: %0.3f milliseconds \n",kernel_read_bandwidth_time / 1000000.0); // converted to miliseconds


  size_t data_transfer_size = sizeof(unsigned char) * size;
  double bandwidth_to_kernel = (double)data_transfer_size / (kernel_write_bandwidth_time * 1.0e-3);
  double bandwidth_from_kernel = (double)data_transfer_size / (kernel_read_bandwidth_time * 1.0e-3);

  printf("Bandwidth to Kernel: %f bytes/s\n", bandwidth_to_kernel);
  printf("Bandwidth from Kernel: %f bytes/s\n", bandwidth_from_kernel);

  size_t total_work = image.width() * image.height() * image.spectrum();
  double throughput = (double)total_work / (kernel_execution_time * 1.0e-3);

  printf("Throughput of the Kernel: %f pixels/s\n", throughput);


  // 11. Check correctness of execution
  // Display the image
  // image_out.display("Image rotation");
  
  // 12. Release OpenCL memory allocated along program
  clReleaseMemObject(in_device_object);
  clReleaseMemObject(out_device_object);
  clReleaseProgram(program);
  clReleaseKernel(kernel);
  clReleaseCommandQueue(command_queue);
  clReleaseContext(context);

  clReleaseEvent(kernel_time);
  clReleaseEvent(kernel_write_bandwidth);
  clReleaseEvent(kernel_read_bandwidth);

  timer = clock() - timer;
  printf("Execution time of the program in seconds: %f s\n", ((float)timer)/CLOCKS_PER_SEC);
  
  return 0;
}

