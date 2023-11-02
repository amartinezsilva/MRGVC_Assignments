#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using my_float = long double;
//using my_float = float; //to compare with kahan implementation


void pi_taylor_chunk(std::vector<my_float> &output, size_t thread_id, size_t start_step, size_t stop_step) {

    auto start = std::chrono::steady_clock::now(); // Timer start

    my_float sum = 0.0;
    int sign = start_step & 0x1 ? -1 : 1;

    for (size_t i = start_step; i < stop_step; i++) {
        sum += sign / static_cast<my_float>(2 * i + 1);
        sign = -sign;
    }

    output[thread_id] = 4 * sum;

    // Timer end and calculation of elapsed time
    auto stop = std::chrono::steady_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Thread " << thread_id << " started at step " << start_step
              << " and ended at step " << stop_step << " with execution time: " << elapsed_seconds.count() << " us\n";
}

std::pair<size_t, size_t>
usage(int argc, const char *argv[]) {
    // read the number of steps from the command line
    if (argc != 3) {
        std::cerr << "Invalid syntax: pi_taylor <steps> <threads>" << std::endl;
        exit(1);
    }

    size_t steps = std::stoll(argv[1]);
    size_t threads = std::stoll(argv[2]);

    if (steps < threads ){
        std::cerr << "The number of steps should be larger than the number of threads" << std::endl;
        exit(1);

    }
    return std::make_pair(steps, threads);
}

typedef struct {
    size_t large_chunk;
    size_t small_chunk;
    size_t split_item;
} chunk_info;

// For a given number of iterations N and threads
// the iterations are divided:
// N % threads receive N / threads + 1 iterations
// the rest receive N / threads
constexpr chunk_info
split_evenly(size_t N, size_t threads)
{
    return {N / threads + 1, N / threads, N % threads};
}


std::pair<size_t, size_t>
get_chunk_begin_end(const chunk_info& ci, size_t index)
{
    size_t begin = 0, end = 0;
    if (index < ci.split_item ) {
        begin = index*ci.large_chunk;
        end = begin + ci.large_chunk; // (index + 1) * ci.large_chunk
    } else {
        begin = ci.split_item*ci.large_chunk + (index - ci.split_item) * ci.small_chunk;
        end = begin + ci.small_chunk;
    }
    return std::make_pair(begin, end);
}

int main(int argc, const char *argv[]) {

    auto start = std::chrono::steady_clock::now();

    auto ret_pair = usage(argc, argv);
    auto steps = ret_pair.first;
    auto threads = ret_pair.second;

    my_float pi;

    std::vector<my_float> output(threads);
    std::vector<std::thread> thread_vector;

    auto chunks = split_evenly(steps, threads);
    
    for(size_t i = 0; i < threads; ++i) {
        auto begin_end = get_chunk_begin_end(chunks, i);
        thread_vector.push_back(std::thread(pi_taylor_chunk, ref(output), i, begin_end.first, begin_end.second));
        //std::cout << i << ", " << begin_end.first << ", " << begin_end.second << std::endl;
    }

    // wait for completion
    for(size_t i = 0; i < threads; ++i) {
        thread_vector[i].join();
    }

    // clean the vector array
    thread_vector.clear();

    //add up the results computed by each thread
    pi = std::accumulate(output.begin(), output.end(), 0.0);

    //Compute elapsed time
    auto stop = std::chrono::steady_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "For " << steps << ", pi value: "
        << std::setprecision(std::numeric_limits<long double>::digits10 + 1)
        << pi << std::endl;

    std::cout << "elapsed time: " << elapsed_seconds.count() << " us\n";

}

