#include <iostream>
#include <Eigen/Dense>

int randomIntNumber(int offset, int range) {
    // Reyrieve a random number between offset and range
	int random = offset + (rand() % range);
    return random;
}
 
int main()
{
  srand((unsigned) time(NULL));

  int range = randomIntNumber(2, 8);

  Eigen::MatrixXd matrix1 = Eigen::MatrixXd::Random(range, range);
  Eigen::MatrixXd matrix2 = Eigen::MatrixXd::Random(range, range);
  Eigen::MatrixXd mult = matrix1 * matrix2;


  std::cout << "matrix1 = \n" << matrix1 << std::endl;
  std::cout << "matrix2 = \n" << matrix2 << std::endl;
  std::cout << "matrix1 * matrix2 =\n" << mult << std::endl;
}