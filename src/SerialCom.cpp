#include "SerialCom.h"
#include <Arduino.h>
#include <atomic>
#include <string>
#include <cstdlib>

// define our inputs from the host device. 
std::atomic<float> desiredRPS(0);
std::atomic<int> desiredSA(0);

bool parseBuffer(char* buffer, size_t end) {
    // In-place trim: find start and end
    size_t start = 0;

    // trim leading and trailing whitespace
    while (start < end && std::isspace(static_cast<unsigned char>(buffer[start]))) ++start;
    while (end > start && std::isspace(static_cast<unsigned char>(buffer[end - 1]))) --end;

    if (buffer[start] != '<' || buffer[end - 1] != '>') return false;

    size_t comma = start + 1;
    while (comma < end - 1 && buffer[comma] != ',') ++comma;
    if (comma >= end - 1) return false; // no comma or nothing after

  char saveComma = buffer[comma];
  char saveEnd   = buffer[end - 1];
  buffer[comma] = '\0';
  buffer[end - 1] = '\0';

  char* endptr = nullptr;
  errno = 0;
  float f = strtof(&buffer[start + 1], &endptr);
  if (errno == ERANGE || endptr == &buffer[start + 1]) { // overflow or no conversion error
     // restore overwritten chars
     buffer[comma] = saveComma;
     buffer[end - 1] = saveEnd;
    return false;
  }

  errno = 0;
  long iv = strtol(&buffer[comma + 1], &endptr, 10);
  if (errno == ERANGE || endptr == &buffer[comma + 1]) { // overflow or no conversion error
     // restore overwritten chars
     buffer[comma] = saveComma;
     buffer[end - 1] = saveEnd;
    return false;
  }

  // restore overwritten chars
  buffer[comma] = saveComma;
  buffer[end - 1] = saveEnd;

  desiredRPS.store(f, std::memory_order_relaxed);
  desiredSA.store((static_cast<int>(iv)), std::memory_order_relaxed);
  return true;
}

// Making this funciton off the main thread. 
// Will stop the motor driver waiting for allocating memory
// Every Signal time this reads. 
void getCurrentCommand(void* param) {
  const int BUFFER_SIZE = 64; 
  char receiverBuffer[BUFFER_SIZE];
  int bufferIndex = 0;

  for (;;) {
    
    while (Serial.available()) {
      int c = Serial.read();
      if (c == -1){
        break;
      }
      char ch = static_cast<char>(c);

      // handle CR/LF: treat '\r' as ignored, '\n' as end-of-frame
      if (ch == '\r') {
        break;
      }

      if (ch == '\n') {
        if (bufferIndex > 0) {
          receiverBuffer[bufferIndex] = '\0';
          parseBuffer(receiverBuffer, (size_t)bufferIndex);
        }
        break;
      } else {
        if (bufferIndex < BUFFER_SIZE - 1) {
          receiverBuffer[bufferIndex++] = ch;
        } else {
          break;
        }
      }
    }

    //Serial.flush(); // wait till the all the outgoing Serial data is finished.
    bufferIndex = 0;

    vTaskDelay(pdMS_TO_TICKS(5)); // give CPU back to other tasks
  }
}


bool readThreeFloats(float &a, float &b, float &c) {
  static String inputLine = "";

  // Check if data is available
  while (Serial.available() > 0) {
    char ch = Serial.read();

    if (ch == '\n') {
      // Try to parse floats when a line is complete
      int numParsed = sscanf(inputLine.c_str(), "%f %f %f", &a, &b, &c);
      inputLine = "";  // reset buffer

      if (numParsed == 3) {
        return true;  // successfully read 3 floats
      } else {
        return false; // parsing failed
      }
    } else {
      inputLine += ch;
    }
  }

  return false; // no complete line yet
}