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
  clock_t timer, execution_timer;
	timer = clock();

  cl_event kernel_time_GPU;
  cl_event kernel_write_bandwidth_CPU;
  cl_event kernel_write_bandwidth_GPU;
  cl_event kernel_read_bandwidth_GPU;

  int err, err_GPU;                            	// error code returned from api calls
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
  cl_context context_GPU;                 				// compute context
  cl_command_queue command_queue_GPU;     				// compute command queue

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


  execution_timer = clock();

  // 3.1 Create a context, with GPU device
  cl_context_properties properties_GPU[] = { CL_CONTEXT_PLATFORM, (cl_context_properties)platforms_ids[2], 0};
  context_GPU = clCreateContext(properties_GPU, n_devices[0],&(devices_ids[2][0]), NULL, NULL, &err_GPU); //REVISAR NUMERO
  cl_error(err_GPU, "Failed to create a compute context for GPU\n");


  // 4.1 Create a command queue for GPU device
  cl_command_queue_properties proprt_GPU[] = { CL_QUEUE_PROPERTIES, CL_QUEUE_PROFILING_ENABLE, 0 };
  command_queue_GPU = clCreateCommandQueueWithProperties( context_GPU, devices_ids[2][0], proprt_GPU, &err_GPU); //REVISAR NUMERO
  cl_error(err_GPU, "Failed to create a command queue for GPU\n");

  // 2. Load source code of kernel

  // Calculate size of the file
  FILE *fileHandler = fopen("kernel_rotation.cl", "r");
  if (fileHandler == NULL) {
    fprintf(stderr, "Failed to open the OpenCL source file\n");
    exit(-1);
  }
  fseek(fileHandler, 0, SEEK_END);
  size_t fileSize = ftell(fileHandler);
  rewind(fileHandler);



  // read kernel source into buffer
  char * sourceCode = (char*) malloc(fileSize + 1);
  sourceCode[fileSize] = '\0';
  fread(sourceCode, sizeof(char), fileSize, fileHandler);
  fclose(fileHandler);


  // create program from buffer for GPU
  cl_program program_GPU = clCreateProgramWithSource(context_GPU, 1, (const char**)&sourceCode, &fileSize, &err_GPU);
  cl_error(err_GPU, "Failed to create program with source for GPU\n");
  free(sourceCode);

  // Same kernel for both devices, so same buffer


  // 3. Build the executable and check errors

  cl_device_id gpuDevice = devices_ids[2][0];

  err_GPU = clBuildProgram(program_GPU, 1, &gpuDevice, NULL, NULL, NULL);

  if (err_GPU != CL_SUCCESS){
    size_t len;
    char buffer[2048];

    printf("Error: Some error at building process of the GPU.\n");
    clGetProgramBuildInfo(program_GPU, devices_ids[2][0], CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
    buffer[len] = '\0';
    printf("%s\n", buffer);
    exit(-1);
  }

  // 4.1 Create a compute kernel with the program we want to run
  cl_kernel kernel_GPU = clCreateKernel(program_GPU, "image_rotation", &err_GPU);
  cl_error(err_GPU, "Failed to create kernel from the program of the GPU\n");


  // 5. Create and initialize input and output arrays at host memory
  CImg<unsigned char> image("image.jpg");  // Load image file "image.jpg" at object img
  // image size
  int width = image.width();
  int height = image.height();
  int spectrum = image.spectrum();

  size_t size = image.size();

  // 6.0 Create OpenCL buffers visible to the OpenCl runtime for the CPU device
  cl_ulong total_memory_allocated_CPU = 0;

  size_t total_images = 3000;

  size_t images_to_GPU = total_images;

  // Replicate image data in memory
  size_t total_size = size * total_images;
  unsigned char* all_images_data = new unsigned char[total_size];

  for (size_t i = 0; i < total_images; ++i) {
      // Copy image data into the replicated block
      std::memcpy(&all_images_data[i * size], image.data(), size);
  }

  unsigned char* image_data_for_GPU = &all_images_data[0]; // Point to the last image data assigned to the CPU

  // 6.1 Create OpenCL buffers visible to the OpenCl runtime for the GPU device
  cl_ulong total_memory_allocated_GPU = 0;

  cl_mem in_device_object_GPU = clCreateBuffer(context_GPU, CL_MEM_READ_ONLY, sizeof(unsigned char)*size * images_to_GPU, NULL, &err_GPU);
  cl_error(err_GPU, "Failed to create memory buffer at GPU device\n");
  cl_ulong in_device_mem_size_GPU;
  clGetMemObjectInfo(in_device_object_GPU, CL_MEM_SIZE, sizeof(cl_ulong), &in_device_mem_size_GPU, NULL);
  total_memory_allocated_GPU += in_device_mem_size_GPU;

  cl_mem out_device_object_GPU = clCreateBuffer(context_GPU, CL_MEM_WRITE_ONLY, sizeof(unsigned char)*size * images_to_GPU, NULL, &err_GPU);
  cl_error(err_GPU, "Failed to create memory buffer at GPU device\n");
  cl_ulong out_device_mem_size_GPU;
  clGetMemObjectInfo(out_device_object_GPU, CL_MEM_SIZE, sizeof(cl_ulong), &out_device_mem_size_GPU, NULL);
  total_memory_allocated_GPU += out_device_mem_size_GPU;
  

  // 7.1 Write data into the memory object for GPU
  err_GPU = clEnqueueWriteBuffer(command_queue_GPU, in_device_object_GPU, CL_TRUE, 0, sizeof(unsigned char)*size * images_to_GPU,
                              image_data_for_GPU, 0, NULL, &kernel_write_bandwidth_GPU);
  cl_error(err_GPU, "Failed to enqueue a write command for GPU\n");

  clWaitForEvents(1, &kernel_write_bandwidth_GPU);

  cl_ulong kernel_write_bandwidth_time_start_GPU, kernel_write_bandwidth_time_end_GPU;
  clGetEventProfilingInfo(kernel_write_bandwidth_GPU, CL_PROFILING_COMMAND_START, sizeof(cl_ulong), &kernel_write_bandwidth_time_start_GPU, NULL);
  clGetEventProfilingInfo(kernel_write_bandwidth_GPU, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &kernel_write_bandwidth_time_end_GPU, NULL);

  double kernel_write_bandwidth_time_GPU = (double)(kernel_write_bandwidth_time_end_GPU - kernel_write_bandwidth_time_start_GPU);


  // 8.1. Set the arguments to the kernel
  err_GPU = clSetKernelArg(kernel_GPU, 0, sizeof(cl_mem), &in_device_object_GPU);
  cl_error(err_GPU, "Failed to set argument 0 in GPU\n");
  err_GPU = clSetKernelArg(kernel_GPU, 1, sizeof(cl_mem), &out_device_object_GPU);
  err_GPU = clSetKernelArg(kernel_GPU, 2, sizeof(int), &width);
  cl_error(err_GPU, "Failed to set argument 1 in GPU\n");
  err_GPU = clSetKernelArg(kernel_GPU, 3, sizeof(int), &height);
  cl_error(err_GPU, "Failed to set argument 2 in GPU\n");
  float angle = 1.570796; //rotate 90 degrees
  err_GPU = clSetKernelArg(kernel_GPU, 4, sizeof(float), &angle);
  cl_error(err_GPU, "Failed to set argument 3 in GPU\n");


  // 9.0 Launch Kernel for CPU

  const size_t global_size[2] = {image.width() , image.height()};
  

  // 9.1 Launch Kernel for GPU

  err_GPU = clEnqueueNDRangeKernel(command_queue_GPU, kernel_GPU, 2, NULL, global_size, NULL, 0, NULL, &kernel_time_GPU);
  cl_error(err_GPU, "Failed to launch kernel to the GPU\n");

  // After the kernel execution
  clWaitForEvents(1, &kernel_time_GPU);

  cl_ulong kernel_time_start_GPU, kernel_time_end_GPU;
  clGetEventProfilingInfo(kernel_time_GPU, CL_PROFILING_COMMAND_START, sizeof(cl_ulong), &kernel_time_start_GPU, NULL);
  clGetEventProfilingInfo(kernel_time_GPU, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &kernel_time_end_GPU, NULL);

  double kernel_execution_time_GPU = (double)(kernel_time_end_GPU - kernel_time_start_GPU); 

  // 10.1 Read data form device memory back to host memory from GPU
  //CImg<unsigned char> image_out_GPU_all(image.width(), image.height(), 1, images_to_GPU);

  size_t data_transfer_size_GPU = sizeof(unsigned char) * size * images_to_GPU;

  // unsigned char* out_images_gpu = new unsigned char[size*images_to_GPU];
  // std::cout << "output data gpu created" << std::endl;
  double kernel_read_bandwidth_time_GPU = 0.0;
  for (int i = 0; i < images_to_GPU; ++i) {

    CImg<unsigned char> image_out(image.width(), image.height(), 1, 3);
    //cl_event kernel_read_bandwidth_GPU;
    err_GPU = clEnqueueReadBuffer(command_queue_GPU, out_device_object_GPU, CL_TRUE, 0,
                                  sizeof(unsigned char) * size, image_out.data(), 0, NULL, &kernel_read_bandwidth_GPU);

    cl_error(err_GPU, "Failed to enqueue a read command for GPU\n");
    clWaitForEvents(images_to_GPU, &kernel_read_bandwidth_GPU);
    cl_ulong kernel_read_bandwidth_time_start_GPU, kernel_read_bandwidth_time_end_GPU;
    clGetEventProfilingInfo(kernel_read_bandwidth_GPU, CL_PROFILING_COMMAND_START, sizeof(cl_ulong), &kernel_read_bandwidth_time_start_GPU, NULL);
    clGetEventProfilingInfo(kernel_read_bandwidth_GPU, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &kernel_read_bandwidth_time_end_GPU, NULL);
    kernel_read_bandwidth_time_GPU += (double)(kernel_read_bandwidth_time_end_GPU - kernel_read_bandwidth_time_start_GPU); 

    // //Check correctness
    // image_out.display("Image rotation");
  }

  double bandwidth_to_kernel_GPU = (double)(data_transfer_size_GPU ) / (kernel_write_bandwidth_time_GPU * 1.0e-3);
  double bandwidth_from_kernel_GPU = (double)(data_transfer_size_GPU) / (kernel_read_bandwidth_time_GPU * 1.0e-3);

  size_t total_work_GPU = image.width() * image.height() * images_to_GPU;
  double throughput_GPU = (double)total_work_GPU / (kernel_execution_time_GPU * 1.0e-3);


  execution_timer = clock() - execution_timer;
  printf("Execution time of the program before releasing memory allocated and prints in seconds: %f s\n", ((float)execution_timer)/CLOCKS_PER_SEC);
  printf("\n");

  printf("Image size: %d\n", image.size());
  printf("Image width: %d\n", width);
  printf("Image heigth: %d\n", height);
  printf("Image spectrum: %d\n", spectrum);

  printf("Memory Footprint of the auxiliar buffer: %zu bytes\n", t_buf * sizeof(char));
  printf("Memory Footprint of the file buffer: %d bytes\n", fileSize);

  printf("Images to process: %d \n", total_images);

  printf("\n");
  printf("Images to GPU: %d \n", images_to_GPU);
  printf("Memory Footprint of the Read/Write buffers for GPU: %lu bytes\n", total_memory_allocated_GPU);
  printf("OpenCL write buffer time for GPU is: %0.3f milliseconds\n", kernel_write_bandwidth_time_GPU / 1000000.0); // converted to milliseconds
  printf("OpenCl Kernel in GPU execution time is: %0.3f milliseconds \n",kernel_execution_time_GPU / 1000000.0); // converted to miliseconds
  printf("OpenCl read buffer for GPU time is: %0.3f milliseconds \n",kernel_read_bandwidth_time_GPU / 1000000.0); // converted to miliseconds
  printf("Bandwidth of GPU to Kernel: %f bytes/ms\n", bandwidth_to_kernel_GPU);
  printf("Bandwidth of GPU from Kernel: %f bytes/ms\n", bandwidth_from_kernel_GPU);
  printf("Throughput of GPU of the Kernel: %f pixels/ms\n", throughput_GPU);

  
  // 12. Release OpenCL memory allocated along program
  
  clReleaseMemObject(in_device_object_GPU);
  clReleaseMemObject(out_device_object_GPU);
  clReleaseProgram(program_GPU);
  clReleaseKernel(kernel_GPU);
  clReleaseCommandQueue(command_queue_GPU);
  clReleaseContext(context_GPU);

  clReleaseEvent(kernel_time_GPU);
  clReleaseEvent(kernel_write_bandwidth_GPU);
  clReleaseEvent(kernel_read_bandwidth_GPU);

  timer = clock() - timer;
  printf("Execution time of the program in seconds: %f s\n", ((float)timer)/CLOCKS_PER_SEC);
  
  delete[] all_images_data;
  // delete[] out_images_cpu;
  // delete[] out_images_gpu;


  return 0;
}

