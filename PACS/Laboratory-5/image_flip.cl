__kernel void image_flip(
  __read_only image2d_t inputImage,
  __write_only image2d_t outputImage){


  const int2 gid = (int2)(get_global_id(0), get_global_id(1));
  
  // Read the pixel from the original image
  uint4 pixel = read_imageui(inputImage, gid);
  
  // Calculate the corresponding index for the vertically flipped pixel
  int flippedX = get_image_width(inputImage) - gid.x - 1;

  // Write the pixel to the flipped position in the image
  write_imageui(outputImage, (int2)(flippedX, gid.y), pixel);
}