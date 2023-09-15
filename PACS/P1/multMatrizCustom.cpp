#include <iostream>
#include <cstdlib>

using namespace std;

//Random does not work fine
//We have to change float by double
//Makes well the functions
//Makes the eigen
//compare

int randomNumber(int offset, int range) {
    // Reyrieve a random number between offset and range
    int random = offset + (rand() % range);
    return random;
}

int main() {
	
	srand((unsigned) time(NULL));

    int range = randomNumber(2, 8);

    int matrix1[range][range];
    int matrix2[range][range];
    int mult[range][range];
    
    cout << "matriz A:" << endl;
    for (int i=0; i<range; i++)
        for (int j=0; j<range; j++)
	{
	matrix1[i][j] = randomNumber(0, 10);
	cout << " " << matrix1[i][j];
	if(j == range-1)
		cout << endl;
	}
    cout << " " << endl;
    cout << "matriz B:" << endl;
    for (int i=0; i<range; i++)
        for (int j=0; j<range; j++)
	{
	matrix2[i][j] = randomNumber(0, 10);
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
