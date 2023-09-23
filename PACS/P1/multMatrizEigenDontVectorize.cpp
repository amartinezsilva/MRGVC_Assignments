#include <iostream>
#include <Eigen/Dense>

//DISABLE VECTORIZATION
#define EIGEN_DONT_VECTORIZE 1

using namespace std;

double randomDoubleNumber(double max, double min) {
    // Retrieve a random number between offset and range
    double random = (max - min) * ((double)rand() / (double)RAND_MAX) + min;
    return random;
}
 
int main()
{
  //Set to true to print results -> NEED TO BE FALSE FOR METRICS!!
  bool debug = false;

  srand((unsigned) time(NULL));

  //prefixed range (for experiments N=10, N=100, N=1000, N=2000)
	int range = 2000;

  Eigen::MatrixXd matrix1(range, range);
  Eigen::MatrixXd matrix2(range, range);

  for (int row = 0; row < range; ++row)
  {
    for (int col = 0; col < range; ++col)
    {
      matrix1(row,col) = randomDoubleNumber(0, 100);
      matrix2(row,col) = randomDoubleNumber(0, 100);
    }
  }
  Eigen::MatrixXd mult = matrix1 * matrix2;

  if(debug){
    std::cout << "matrix1 = \n" << matrix1 << std::endl;
    std::cout << "matrix2 = \n" << matrix2 << std::endl;
    std::cout << "matrix1 * matrix2 =\n" << mult << std::endl;
  }
}