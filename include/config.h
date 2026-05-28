
// #define ENABLE_SERIAL_DEBUG  

// #define ESPNOW
#define UDP

#define PID_250
// #define PID_1000

#define CF
// #define KARMAL

#define MAIN
// #define TEST_PID
// #define TEST_MOTOR_VIA_SERIAL
// #define TEST_MOTOR_VIA_ESPNOW
// #define TEST_PIN
// #define TEST_ESPNOW



#ifdef ENABLE_SERIAL_DEBUG
  #define SERIAL_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
  #define SERIAL_BEGIN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(format, ...) 
#endif
