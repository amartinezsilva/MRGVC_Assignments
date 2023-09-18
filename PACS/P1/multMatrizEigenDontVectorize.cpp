#include <iostream>
#include <Eigen/Dense>
#include <chrono>

#define EIGEN_DONT_VECTORIZE

using namespace std;

int randomIntNumber(int offset, int range) {
    // Reyrieve a random number between offset and range
	int random = offset + (rand() % range);
    return random;
}
 
int main()
{
  
  auto start = chrono::high_resolution_clock::now();

	//Set to true to print results -> NEED TO BE FALSE FOR METRICS!!
  bool debug = false;

  srand((unsigned) time(NULL));

  int range = randomIntNumber(2, 998);
  //prefixed range (for experiments N=10, N=100, N=1000)
	//int range = 2;

  Eigen::MatrixXd matrix1 = Eigen::MatrixXd::Random(range, range)*100.0;
  Eigen::MatrixXd matrix2 = Eigen::MatrixXd::Random(range, range)*100.0;
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