#include <iostream>
#include <math.h>

float triangleAreaFromVertices(float v[]){
	// TO-DO: Calculate and return the triangle area using the vertex coordinates in v[]

	float s1 = v[0]*(v[3] - v[5]);
	float s2 = v[2]*(v[5] - v[1]);
	float s3 = v[4]*(v[1] - v[3]);

	float area = 1/2.0 * abs(s1 + s2 + s3);
	return area;
}

int main() {
	// Now you can give the triangle the values on the code itself
	float v[] = {1.0f, 0.0f, 3.0f, 0.0f, 2.0f, 2.0f};
	
	
	std::cout << "The area of the triangle is " << triangleAreaFromVertices(v) << std::endl;
		
	/* TO-DO: Once you tested the example above, create a new array of nine floats, 
	and try to assign the six coordinates of the triangle vertices, the triangle area, 
	the number of vertices of the triangle, and a string with the triangle name (e.g., “Triangle”) */ 
	
    return 0;
}