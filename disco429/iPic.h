#pragma once
//{{{  includes
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
//}}}

class iPic {
public:
  virtual ~iPic() {}

  virtual uint16_t getWidth() = 0;
  virtual uint16_t getHeight() = 0;
  virtual uint16_t getComponents() = 0;
  virtual uint8_t* getPic() = 0;

  virtual void setPic (uint8_t* buffer, int size) = 0;
  };
