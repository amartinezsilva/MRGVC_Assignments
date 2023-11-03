#include <iomanip>
#include <iostream>
#include <limits>
#include <chrono>


// Allow to change the floating point type
using my_float = long double;

my_float pi_taylor(size_t steps) {

    my_float sum = 0.0;
    int sign = 1;
    for(size_t i = 0; i < steps; i++){
        sum+= sign / static_cast<float>(2*i + 1);
        sign = -sign;
    }

    return 4*sum;
}

int main(int argc, const char *argv[]) {

    //std::chrono::time_point<std::chrono::system_clock> start, end;

    auto start = std::chrono::steady_clock::now();

    // read the number of steps from the command line
    if (argc != 2) {
        std::cerr << "Invalid syntax: pi_taylor <steps>" << std::endl;
        exit(1);

    }

    size_t steps = std::stoll(argv[1]);
    auto pi = pi_taylor(steps);

    std::cout << "For " << steps << ", pi value: "
        << std::setprecision(std::numeric_limits<my_float>::digits10 + 1)
        << pi << std::endl;

    auto end = std::chrono::steady_clock::now();

    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
 
    std::cout << "elapsed time (ms): " << elapsed_time.count() << " us\n";

}
