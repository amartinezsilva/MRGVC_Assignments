#include <iostream>
#include <Eigen/Dense>
#include <sys/time.h>
#include <cmath>

using namespace std;

double randomDoubleNumber(double max, double min) {
    // Retrieve a random number between offset and range
    double random = (max - min) * ((double)rand() / (double)RAND_MAX) + min;
    return random;
}
 
int main()
{
  struct timeval timestampIni;
  gettimeofday(&timestampIni, NULL);
  //cout << "At the beggining, Seconds: " << timestampIni.tv_sec << endl
  //<< "Microseconds: " << timestampIni.tv_usec << endl;

  //Set to true to print results -> NEED TO BE FALSE FOR METRICS!!
  bool debug = false;

  srand((unsigned) time(NULL));

  //prefixed range (for experiments N=10, N=100, N=1000, N=2000)
	int range = 1000;

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

  struct timeval timestampBef;
  gettimeofday(&timestampBef, NULL);
  //cout << "Before the multiplication, Seconds: " << timestampBef.tv_sec << endl
  //<< "Microseconds: " << timestampBef.tv_usec << endl;

  Eigen::MatrixXd mult = matrix1 * matrix2;

  struct timeval timestampAf;
  gettimeofday(&timestampAf, NULL);
  //cout << "After the multiplication, Seconds: " << timestampAf.tv_sec << endl
  //<< "Microseconds: " << timestampAf.tv_usec << endl;

  if(debug){
    std::cout << "matrix1 = \n" << matrix1 << std::endl;
    std::cout << "matrix2 = \n" << matrix2 << std::endl;
    std::cout << "matrix1 * matrix2 =\n" << mult << std::endl;
  }

  struct timeval timestampEnd;
  gettimeofday(&timestampEnd, NULL);
  //cout << "At the end, Seconds: " << timestampEnd.tv_sec << endl
  //<< "Microseconds: " << timestampEnd.tv_usec << endl;

	struct timeval timestampMult;
	timestampMult.tv_sec = timestampAf.tv_sec- timestampBef.tv_sec;
	timestampMult.tv_usec = timestampAf.tv_usec- timestampBef.tv_usec;
	cout << "Multiplication time, Seconds: " << timestampMult.tv_sec << endl
	<< "Microseconds: " << timestampMult.tv_usec << endl;

		struct timeval timestampExtra;
	timestampExtra.tv_sec = (timestampBef.tv_sec - timestampIni.tv_sec) + (timestampEnd.tv_sec - timestampAf.tv_sec);
	timestampExtra.tv_usec = (timestampBef.tv_usec - timestampIni.tv_usec) + (timestampEnd.tv_usec - timestampAf.tv_usec);
	cout << "NO multiplication time, Seconds: " << timestampExtra.tv_sec << endl
	<< "Microseconds: " << timestampExtra.tv_usec << endl;

}