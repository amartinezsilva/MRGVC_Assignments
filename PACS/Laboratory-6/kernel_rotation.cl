__kernel void image_rotation(
  __global const uchar4 *img_input,
  __global uchar4 *img_output,
  const int width,
  const int height,
  const float angle){

    const int x1 = get_global_id(0);
    const int y1 = get_global_id(1);
    const int rgb = get_global_id(2);

    const int offset = width * height * rgb;
    const int index = offset + x1 + y1 * width;

    const int x0 = width/2;
    const int y0 = height/2;

    const int x2 = cos(angle) * (x1-x0) - sin(angle) * (y1-y0) + x0;
    const int y2 = sin(angle) * (x1-x0) - cos(angle) * (y1-y0) + y0;

    const int output_index = offset + x2 + y2 * width;
    
    img_output[output_index] = img_input[index];
}