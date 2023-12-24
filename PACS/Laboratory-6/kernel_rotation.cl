__kernel void image_rotation(
  __global const uchar *img_input,
  __global uchar *img_output,
  const int width,
  const int height,
  const float angle){

    const int x1 = get_global_id(0);
    const int y1 = get_global_id(1);

    const int indexR = x1 + y1 * width;
    const int indexG = indexR + width*height;
    const int indexB = indexG + width*height;

    const int x0 = width/2;
    const int y0 = height/2;

    const int x2 = cos(angle) * (x1-x0) - sin(angle) * (y1-y0) + x0;
    const int y2 = sin(angle) * (x1-x0) - cos(angle) * (y1-y0) + y0;

    const int output_indexR = x2 + y2 * width;
    const int output_indexG = output_indexR + width*height;
    const int output_indexB = output_indexG + width*height;

    
    img_output[output_indexR] = img_input[indexR];
    img_output[output_indexG] = img_input[indexG];
    img_output[output_indexB] = img_input[indexB];

}