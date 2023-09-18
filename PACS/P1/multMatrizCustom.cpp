#include <iostream>
#include <cstdlib>
#include <chrono>

using namespace std;

//Makes well the functions
//Dynamic alloc
//Makes the eigen
//compare

double randomDoubleNumber(double max, double min) {
    // Reyrieve a random number between offset and range
    double random = (max - min) * ((double)rand() / (double)RAND_MAX) + min;
    return random;
}

int randomIntNumber(int offset, int range) {
    // Reyrieve a random number between offset and range
	int random = offset + (rand() % range);
    return random;
}

int main() {
	
	auto start = chrono::high_resolution_clock::now();

	//Set to true to print results -> NEED TO BE FALSE FOR METRICS!!
	bool debug = false;

	srand((unsigned) time(NULL));

	//random range
	//int range = randomIntNumber(2, 998);
	//prefixed range (for experiments N=10, N=100, N=1000)
	int range = 2000;

	double * matrix1;
	double * matrix2;
	double * mult;

	matrix1 = (double*) calloc (range*range, sizeof(double));
	matrix2 = (double*) calloc (range*range, sizeof(double));
	mult = (double*) calloc (range*range, sizeof(double));
    
	//Autofill matrix 1 and 2 with random double numbers
	for (int i=0; i<range*range; i++){
		matrix1[i] = randomDoubleNumber(0, 100);
		matrix2[i] = randomDoubleNumber(0, 100);
	}

	//Perform multiplication
	for (int i=0; i<range; i++)
        for (int j=0; j<range; j++)
		{
			for (int k=0; k<range; k++) {
				mult[i*range + j]+= matrix1[i*range + k] * matrix2[k*range + j];
			}
		}

	//Print matrices A and B
	if(debug){
		cout << "matriz A:" << endl;
		for (int i=0; i<range; i++)
			for (int j=0; j<range; j++)
			{
				cout << " " << matrix1[i*range + j];
				if(j == range-1)
					cout << endl;
			}
		cout << " " << endl;
		cout << "matriz B:" << endl;
		for (int i=0; i<range; i++)
			for (int j=0; j<range; j++)
			{
				cout << " " << matrix2[i*range+j];
				if(j == range-1)
					cout << endl;
			}

		cout << " " << endl;
		cout << "multiplication:" << endl;
		for (int i=0; i<range; i++)
			for (int j=0; j<range; j++)
		{
			cout << " " << mult[i*range + j];
			if(j == range-1)
				cout << endl;
		}
	}

	free(matrix1);
	free(matrix2);
	free(mult);

	auto stop = chrono::high_resolution_clock::now();
	auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
 
    cout << "N = "<< range << ". Time taken by function: "
         << duration.count() << " microseconds" << endl;

    return 0;
}
