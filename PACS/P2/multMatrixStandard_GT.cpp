#include <iostream>
#include <cstdlib>
#include <sys/time.h>

using namespace std;

double randomDoubleNumber(double max, double min) {
    // Retrieve a random number between offset and range
    double random = (max - min) * ((double)rand() / (double)RAND_MAX) + min;
    return random;
}

void showMatrix(double * matrix, int range) {
    for (int i=0; i<range; i++)
			for (int j=0; j<range; j++)
			{
				cout << " " << matrix[i*range + j];
				if(j == range-1)
					cout << endl;
			}
}

int main() {
	struct timeval timestampIni;
	gettimeofday(&timestampIni, NULL);
	//cout << "At the beggining, Seconds: " << timestampIni.tv_sec << endl
	//<< "Microseconds: " << timestampIni.tv_usec << endl;
	
	//Set to true to print results -> NEED TO BE FALSE FOR METRICS!!
	bool debug = false;

	srand((unsigned) time(NULL));

	//prefixed range (for experiments N=10, N=100, N=1000, N=2000)
	int range = 1000;

	double * matrixA;
	double * matrixB;
	double * mult;

	matrixA = (double*) calloc (range*range, sizeof(double));
	matrixB = (double*) calloc (range*range, sizeof(double));
	mult = (double*) calloc (range*range, sizeof(double));
    
	//Autofill matrix A and B with random double numbers
	for (int i=0; i<range*range; i++){
		matrixA[i] = randomDoubleNumber(0, 100);
		matrixB[i] = randomDoubleNumber(0, 100);
	}

	struct timeval timestampBef;
	gettimeofday(&timestampBef, NULL);
  	//cout << "Before the multiplication, Seconds: " << timestampBef.tv_sec << endl
  	//<< "Microseconds: " << timestampBef.tv_usec << endl;

	//Perform multiplication
	for (int i=0; i<range; i++)
        for (int j=0; j<range; j++)
		{
			for (int k=0; k<range; k++) {
				mult[i*range + j]+= matrixA[i*range + k] * matrixB[k*range + j];
			}
		}

	struct timeval timestampAf;
	gettimeofday(&timestampAf, NULL);
  	//cout << "After the multiplication, Seconds: " << timestampAf.tv_sec << endl
  	//<< "Microseconds: " << timestampAf.tv_usec << endl;

	//Print matrices A and B
	if(debug){
		cout << "matriz A:" << endl;
		showMatrix(matrixA, range);

		cout << " " << endl;
		cout << "matriz B:" << endl;
		showMatrix(matrixB, range);

		cout << " " << endl;
		cout << "multiplication:" << endl;
		showMatrix(mult, range);
	}

	free(matrixA);
	free(matrixB);
	free(mult);

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
    return 0;
}
