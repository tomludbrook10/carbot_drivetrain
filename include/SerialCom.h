#pragma once

#include <atomic>
#include <string>
#include <cstdlib>

// declareation here.
extern std::atomic<float> desiredRPS;
extern std::atomic<int> desiredSA;

/// code for parse the input buffer.
// rougly 30bytes are used. 10 for variables and 

bool parseBuffer(char* buffer, size_t end);
// Making this funciton off the main thread. 
// Will stop the motor driver waiting for allocating memory
// Every Signal time this reads. 
void getCurrentCommand(void* param);

bool readThreeFloats(float &a, float &b, float &c);