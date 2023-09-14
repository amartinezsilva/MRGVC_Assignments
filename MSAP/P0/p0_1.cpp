#include <iostream>
#include <math.h>

float euclideanDistance(float x1, float y1, float x2, float y2){
	// TO-DO: Calculate and return the Euclidean distance between (x1,y1) and (x2,y2) 
    float delta_x = x2 - x1;
    float delta_y = y2 - y1;
    float distance = sqrt(delta_x*delta_x + delta_y*delta_y);
	return distance;
}

int main() {
	float x1, y1, x2, y2;
    std::cout << "Introduce x and y coordinates (sep. by space): ";
    std::cin >> x1 >> y1;
    std::cout << "Introduce (another) x and y coordinates (sep. by space): ";
    std::cin >> x2 >> y2;
    std::cout << "The distance between coordinates is " << euclideanDistance(x1,y1,x2,y2) << std::endl;
    return 0;
}