// ps2.cpp
#include "ps2.h"

//{{{  struct ps2Keymap_t
typedef struct {
  uint8_t noshift[PS2_KEYMAP_SIZE];
  uint8_t shift[PS2_KEYMAP_SIZE];
  uint8_t uses_altgr;
  uint8_t altgr[PS2_KEYMAP_SIZE];
  } ps2Keymap_t;
//}}}
//{{{
const ps2Keymap_t kPs2Keymap = {
  // without shift
  {0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
   0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '`', 0,
   0, 0 /*Lalt*/, 0 /*Lshift*/, 0, 0 /*Lctrl*/, 'q', '1', 0,
   0, 0, 'z', 's', 'a', 'w', '2', 0,
   0, 'c', 'x', 'd', 'e', '4', '3', 0,
   0, ' ', 'v', 'f', 't', 'r', '5', 0,
   0, 'n', 'b', 'h', 'g', 'y', '6', 0,
   0, 0, 'm', 'j', 'u', '7', '8', 0,
   0, ',', 'k', 'i', 'o', '0', '9', 0,
   0, '.', '/', 'l', ';', 'p', '-', 0,
   0, 0, '\'', 0, '[', '=', 0, 0,
   0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER /*Enter*/, ']', 0, '\\', 0, 0,
   0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
   0, '1', 0, '4', '7', 0, 0, 0,
   '0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
   PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
   0, 0, 0, PS2_F7 },

  // with shift
  {0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
   0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '~', 0,
   0, 0 /*Lalt*/, 0 /*Lshift*/, 0, 0 /*Lctrl*/, 'Q', '!', 0,
   0, 0, 'Z', 'S', 'A', 'W', '@', 0,
   0, 'C', 'X', 'D', 'E', '$', '#', 0,
   0, ' ', 'V', 'F', 'T', 'R', '%', 0,
   0, 'N', 'B', 'H', 'G', 'Y', '^', 0,
   0, 0, 'M', 'J', 'U', '&', '*', 0,
   0, '<', 'K', 'I', 'O', ')', '(', 0,
   0, '>', '?', 'L', ':', 'P', '_', 0,
   0, 0, '"', 0, '{', '+', 0, 0,
   0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER /*Enter*/, '}', 0, '|', 0, 0,
   0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
   0, '1', 0, '4', '7', 0, 0, 0,
   '0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
   PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
   0, 0, 0, PS2_F7 },

  0
  };
//}}}

//{{{  ps2 vars
int bitPos = -1;
uint16_t data = 0;
bool raw = true;
bool ps2rx = true;

volatile int inPtr = 0;
volatile int outPtr = 0;
int rxData[32];

bool rxExpandCode = false;
bool rxReleaseCode = false;
bool shifted = false;
bool ctrled = false;

bool stream = false;
int streamByte = -1;
uint8_t streamBytes[6];

int touchX = 0;
int touchY = 0;
int touchZ = 0;
//}}}
extern "C" { void EXTI2_IRQHandler(); }
//{{{
void EXTI2_IRQHandler() {

  if (__HAL_GPIO_EXTI_GET_IT (GPIO_PIN_2) != RESET) {
    __HAL_GPIO_EXTI_CLEAR_IT (GPIO_PIN_2);

    if (ps2rx) {
      bool bit = (GPIOG->IDR & GPIO_PIN_3) != 0;

      if (bitPos == -1) {
        //{{{  wait for lo start bit
        if (!bit) {
          // lo start bit
          bitPos = 0;
          data = 0;
          }
        }
        //}}}
      else if (bitPos < 8) {
        // get data bits 0..7
        data = data | (bit << bitPos);
        bitPos++;
        }
      else if (bitPos == 8) {
       //{{{  parity bit - got data
       if (stream) {
         if (streamByte == -1) {
           if ((data & 0xC0) == 0x80) {
             streamByte = 0;
             streamBytes[streamByte] =  data;
             }
           }
         else {
           streamByte++;
           if ((streamByte == 3) && ((data & 0xc0) != 0xc0))
             streamByte = -1;
           else {
             streamBytes[streamByte] = data;
              if (streamByte == 5) {
                touchX = ((streamBytes[3] & 0x10) << 8) | ((streamBytes[1] & 0x0F) << 8) | streamBytes[4];
                touchY = ((streamBytes[3] & 0x20) << 7) | ((streamBytes[1] & 0xF0) << 4) | streamBytes[5];
                touchZ = streamBytes[2];
                streamByte = -1;
                }
              }
           }
         }
       else if (raw) {
         rxData[inPtr] = data | (0x100 * rxReleaseCode);
         inPtr = (inPtr + 1) % 32;
         }
       else if (data == 0xE0)
         rxExpandCode = true;
       else if (data == 0xF0)
         rxReleaseCode = true;
       else if (data == 0x12) // SHIFT_L;
         shifted = !rxReleaseCode;
       else if (data == 0x59) // SHIFT_R;
         shifted = !rxReleaseCode;
       else if (data == 0x14) // CTRL_L
         ctrled = !rxReleaseCode;
       else {
         if (rxExpandCode) {
           if (data == 0x70)
             data = PS2_INSERT;
           else if (data == 0x6C)
             data = PS2_HOME;
           else if (data == 0x7D)
             data = PS2_PAGEUP;
           else if (data == 0x71)
             data = PS2_DELETE;
           else if (data == 0x6C)
             data = PS2_HOME;
           else if (data == 0x69)
             data = PS2_END;
           else if (data == 0x6C)
             data = PS2_PAGEDOWN;
           else if (data == 0x75)
             data = PS2_UPARROW;
           else if (data == 0x6B)
             data = PS2_LEFTARROW;
           else if (data == 0x72)
             data = PS2_DOWNARROW;
           else if (data == 0x74)
             data = PS2_RIGHTARROW;
           else if (data == 0x4A)
             data = '/';
           else if (data == 0x5A)
             data = PS2_ENTER;
           else
             data |= 0x200;
           }
         else if (shifted)
           data = kPs2Keymap.shift[data];
         else
           data = kPs2Keymap.noshift[data];

         rxData[inPtr] = data | (0x100 * rxReleaseCode);
         inPtr = (inPtr + 1) % 32;
         rxExpandCode = false;
         rxReleaseCode = false;
         }

       bitPos++;
       }
       //}}}
      else if (bitPos == 9) {
        //{{{  expect hi stop bit
        if (bit)
          bitPos = -1;
        }
        //}}}
      }
    }
  }
//}}}

//{{{
uint16_t ps2get() {
  while (inPtr == outPtr) {}
  uint16_t ch = rxData[outPtr];
  outPtr = (outPtr + 1) % 32;
  return ch;
  }
//}}}
//{{{
void ps2send (uint8_t value)  {

  HAL_GPIO_WritePin (GPIOG, GPIO_PIN_2, GPIO_PIN_RESET); // set clock lo, release inhibit, if necessary
  HAL_Delay (2); // Wait out any final clock pulse, 100us

  ps2rx = false;
  HAL_GPIO_WritePin (GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); // set data lo, start bit
  HAL_GPIO_WritePin (GPIOG, GPIO_PIN_2, GPIO_PIN_SET);   // set clock hi, float

  uint8_t parity = 1;
  for (int bit = 0; bit < 8; bit++) {
    while (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2)) {} // wait for rising edge
    HAL_GPIO_WritePin (GPIOG, GPIO_PIN_3, (value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // set data to i’th data bit
    parity = parity + value;  // Accumulate parity
    value = value >> 1;       // Shift right to get next bit
    while (!HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2)) {} // wait for falling edge
    }

  while (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2)) {} // wait for rising edge
  HAL_GPIO_WritePin (GPIOG, GPIO_PIN_3, (parity & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // set data to parity bit
  while (!HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2)) {} // wait for falling edge

  while (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2)) {} // wait for rising edge
  HAL_GPIO_WritePin (GPIOG, GPIO_PIN_3, GPIO_PIN_SET); // set data hi, stop bit
  while (!HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2)) {} // wait for falling edge

  while (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2)) {} // wait for rising edge
  //if (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2) == true)
  //  lcd->info ("ps2send - missing line control bit");
  while (!HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2)) {} // wait for falling edge
  ps2rx = true;

  //if (ps2get() != 0xFA)
  //  lcd->info ("ps2send - no 0xFA ack");
  }
//}}}
//{{{
void ps2sendTouchpadSpecialCommand (uint8_t arg) {
// send touchpad special command sequence

  for (int i = 0; i < 4; i++) {
    ps2send (0xE8);   // touchpad setResolution
    ps2send ((arg >> (6-2*i)) & 3);
    }
  }
//}}}
//{{{
void initPs2gpio() {

  bitPos = -1;
  inPtr = 0;
  outPtr = 0;

  __HAL_RCC_GPIOG_CLK_ENABLE();

  // PS2 clock
  GPIO_InitTypeDef GPIO_Init_Structure;
  GPIO_Init_Structure.Mode = GPIO_MODE_IT_RISING;
  GPIO_Init_Structure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_Init_Structure.Pull = GPIO_PULLUP;
  GPIO_Init_Structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init (GPIOG, &GPIO_Init_Structure);

  // PS2 data
  GPIO_Init_Structure.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_Init_Structure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_Init_Structure.Pull = GPIO_PULLUP;
  GPIO_Init_Structure.Pin = GPIO_PIN_3;
  HAL_GPIO_Init (GPIOG, &GPIO_Init_Structure);

  // Enable and set EXTI line 2 Interrupt to the lowest priority
  HAL_NVIC_SetPriority (EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ (EXTI2_IRQn);

  HAL_GPIO_WritePin (GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin (GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay (100);
  }
//}}}
//{{{
void initPs2keyboard() {

  ps2send (0xFF);
  //if (ps2get() != 0xAA)
  //  lcd->info ("initPs2keyboard - missing 0xAA reset");

  for (int i = 0; i < 8; i++) {
    ps2send (0xED); // send leds
    ps2send (i);
    HAL_Delay (100);
    }

  ps2send (0x0F2); // sendId
  //lcd->info ("keyboard id " + hex (ps2get()) + hex(ps2get()));
  }
//}}}
//{{{
void initPs2touchpad() {

  ps2send (0xFF);  // Touchpad reset
  //if (ps2get() != 0xAA)
  //  lcd->info ("initPs2touchpad - missing 0xAA reset");
  //if (ps2get() != 0x00)
  //  lcd->info ("initPs2touchpad - missing 0x00 reset");

  //ps2sendTouchpadSpecialCommand (0x00);
  //ps2send (0xE9); // touchpad statusRequest prefixed by specialCommand
  //auto minor = ps2get();
  //ps2get();  // 0x47
  //auto major = ps2get();
  //lcd->info ("Identify " + hex (major & 0x0F) + "." + hex (minor) + " modelCode:" + hex (major >> 4));

  //ps2sendTouchpadSpecialCommand (0x02);
  //ps2send (0xE9); // touchpad statusRequest prefixed by specialCommand
  //auto capMsb = ps2get();
  //ps2get();  // 0x47
  //auto capLsb = PS2get();
  //lcd->info ("Capabilities " + hex ((capMsb << 8) | capLsb));

  //ps2sendTouchpadSpecialCommand (0x03);
  //ps2send (0xE9); // touchpad statusRequest prefixed by specialCommand
  //ps2send modelId1 = ps2get();
  //auto modelId2 = ps2get();
  //auto modelId3 = ps2get();
  //lcd->info ("ModelId " + hex ((modelId1 << 16) | (modelId2 << 8) | modelId3));

  ps2sendTouchpadSpecialCommand (0x80);
  ps2send (0xF3); // touchpad setSampleRate prefixed by specialCommand
  ps2send (0x14); // - setSampleRate = 20

  ps2send (0xF4); // touchpad enable streaming

  stream = true;
  }
//}}}
