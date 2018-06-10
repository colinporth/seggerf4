#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#define bigMalloc(size,tag)    pvPortMalloc(size)
#define bigFree                vPortFree
#define smallMalloc(size,tag)  malloc(size)
#define smallFree              free

#include "heap.h"

//{{{
template <typename T> std::string hex (T value, uint16_t width = 0) {

  std::ostringstream os;
  os << std::hex << std::setfill ('0') << std::setw (width) << value;
  return os.str();
  }
//}}}
//{{{
template <typename T> std::string dec (T value, uint16_t width = 0, char fill = ' ') {

  std::ostringstream os;
  os << std::setfill (fill) << std::setw (width) << value;
  return os.str();
  }
//}}}
