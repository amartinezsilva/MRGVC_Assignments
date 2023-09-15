#include <iostream>
#include <cstdlib>

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
	
	srand((unsigned) time(NULL));

    int range = randomIntNumber(2, 8);

    double matrix1[range][range];
    double matrix2[range][range];
    double mult[range][range];
    
    cout << "matriz A:" << endl;
    for (int i=0; i<range; i++)
        for (int j=0; j<range; j++)
	{
	matrix1[i][j] = randomDoubleNumber(0, 100);
	cout << " " << matrix1[i][j];
	if(j == range-1)
		cout << endl;
	}
    cout << " " << endl;
    cout << "matriz B:" << endl;
    for (int i=0; i<range; i++)
        for (int j=0; j<range; j++)
	{
	matrix2[i][j] = randomDoubleNumber(0, 100);
	cout << " " << matrix2[i][j];
	if(j == range-1)
		cout << endl;
	}

    for (int i=0; i<range; i++)
        for (int j=0; j<range; j++)
	{
	mult[i][j] = 0;
	}

    cout << " " << endl;
    cout << "multiplication:" << endl;
    for (int i=0; i<range; i++)
        for (int j=0; j<range; j++)
	{
	    for (int k=0; k<range; k++)
	    {
	    mult[i][j] += matrix1[i][k] * matrix2[k][j];
	    }
	cout << " " << mult[i][j];
	if(j == range-1)
	    cout << endl;
	}

    return 0;
}
