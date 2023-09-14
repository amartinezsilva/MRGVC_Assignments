#include <iostream>
#include <math.h>

float triangleAreaFromVertices(float vertex_1_x, float vertex_1_y, 
								float vertex_2_x, float vertex_2_y, 
								float vertex_3_x, float vertex_3_y){
	// TO-DO: Calculate and return the triangle area from the input vertices
	float s1 = vertex_1_x*(vertex_2_y - vertex_3_y);
	float s2 = vertex_2_x*(vertex_3_y - vertex_1_y);
	float s3 = vertex_3_x*(vertex_1_y - vertex_2_y);

	float area = 1/2.0 * abs(s1 + s2 + s3);

	return area;
}

int main() {
	// Now you can give the triangle the values on the code itself
	float vertex_1_x = 1.0f;
	float vertex_1_y = 0.0f;
	float vertex_2_x = 3.0f;
	float vertex_2_y = 0.0f;
	float vertex_3_x = 2.0f;
	float vertex_3_y = 2.0f;
	std::cout << "The area of the triangle is " 
				<< triangleAreaFromVertices(vertex_1_x, vertex_1_y, vertex_2_x, vertex_2_y, vertex_3_x, vertex_3_y) 
				<< std::endl;


    return 0;
}