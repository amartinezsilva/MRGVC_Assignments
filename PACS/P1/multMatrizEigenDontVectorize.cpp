#include <iostream>
#include <Eigen/Dense>
#include <chrono>

//DOES NOT WORK -> USE -DEIGEN_DONT_VECTORIZE WHEN COMPILING
#define EIGEN_DONT_VECTORIZE

using namespace std;

int randomIntNumber(int offset, int range) {
    // Reyrieve a random number between offset and range
	int random = offset + (rand() % range);
    return random;
}

double randomDoubleNumber(double max, double min) {
    // Reyrieve a random number between offset and range
    double random = (max - min) * ((double)rand() / (double)RAND_MAX) + min;
    return random;
}
 
int main()
{
  
  auto start = chrono::high_resolution_clock::now();

	//Set to true to print results -> NEED TO BE FALSE FOR METRICS!!
  bool debug = false;

  srand((unsigned) time(NULL));

  //int range = randomIntNumber(2, 998);
  //prefixed range (for experiments N=10, N=100, N=1000)
	int range = 2000;

  // Initialize to random values -> commented as we use same policy as before
  // Eigen::MatrixXd matrix1 = Eigen::MatrixXd::Random(range, range)*100.0;
  // Eigen::MatrixXd matrix2 = Eigen::MatrixXd::Random(range, range)*100.0;

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

  auto stop = chrono::high_resolution_clock::now();
	auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
 
  cout << "N = "<< range << ". Time taken by function: "
         << duration.count() << " microseconds" << endl;
}