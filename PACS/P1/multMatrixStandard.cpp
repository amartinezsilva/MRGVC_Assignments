#include <iostream>
#include <cstdlib>

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
	
	//Set to true to print results -> NEED TO BE FALSE FOR METRICS!!
	bool debug = true;

	srand((unsigned) time(NULL));

	//prefixed range (for experiments N=10, N=100, N=1000, N=2000)
	int range = 2000;

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

	//Perform multiplication
	for (int i=0; i<range; i++)
        for (int j=0; j<range; j++)
		{
			for (int k=0; k<range; k++) {
				mult[i*range + j]+= matrixA[i*range + k] * matrixB[k*range + j];
			}
		}

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

    return 0;
}
