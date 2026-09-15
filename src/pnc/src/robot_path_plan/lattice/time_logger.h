#ifndef TIME_LOGGER_H
#define TIME_LOGGER_H
#include <chrono>

class TimeLogger {
public:
    TimeLogger() = default;
    ~TimeLogger() = default;
    void start() {
        start_ = std::chrono::high_resolution_clock::now();
    }
    double duration() {
        auto end = std::chrono::high_resolution_clock::now();
        double esplase = std::chrono::duration<double, std::ratio<1, 1000>>(end - start_).count();
        return esplase;
    }

private:
    std::chrono::high_resolution_clock::time_point start_;
};
#endif