#pragma once

#define DEBUG_MODE 1  // Bật = 1, Tắt = 0

#if DEBUG_MODE

  #define DEBUG_BEGIN(baud)       Serial.begin(baud)
  #define DEBUG_WAIT()            while (!Serial)
  #define DEBUG_DELAY(ms)         delay(ms)
  #define DEBUG_FLUSH()           Serial.flush()
  #define DEBUG_PRINTF(...)       Serial.printf(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)      Serial.println(__VA_ARGS__)

  // Xử lý macro cho 1 hoặc 2 tham số
  #define GET_MACRO(_1, _2, NAME, ...) NAME
  #define DEBUG_PRINT1(x)         Serial.print(x)
  #define DEBUG_PRINT2(x, base)   Serial.print(x, base)
  #define DEBUG_PRINT(...)        GET_MACRO(__VA_ARGS__, DEBUG_PRINT2, DEBUG_PRINT1)(__VA_ARGS__)

#else

  #define DEBUG_BEGIN(baud)
  #define DEBUG_WAIT()
  #define DEBUG_DELAY(ms)
  #define DEBUG_FLUSH()
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINT(...)

#endif

// Một số macro tiện dụng
#define DEBUG_INFO(x)    DEBUG_PRINT("[INFO] ");  DEBUG_PRINTLN(x)
#define DEBUG_WARN(x)    DEBUG_PRINT("[WARN] ");  DEBUG_PRINTLN(x)
#define DEBUG_ERROR(x)   DEBUG_PRINT("[ERROR] "); DEBUG_PRINTLN(x)
