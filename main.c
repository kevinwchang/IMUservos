//#define DEBUG

#include <pololu/orangutan.h>
#include "twi.h"
#include "sensors.h"
#include "dcm.h"
#include <math.h>

#ifdef DEBUG
#include <stdio.h>
#include <string.h>
#endif

#define ROLL_SERVO  0
#define PITCH_SERVO 1
#define ROLL_SERVO_PIN  IO_C0
#define PITCH_SERVO_PIN IO_C1

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

int main(void)
{
  uint8_t m = 0, last_m = 0, elapsed;
  int16_t roll_p = 0, roll_d, roll_last_p, roll_delta, pitch_p = 0, pitch_d, pitch_last_p, pitch_delta;
  int32_t roll_i = 0, pitch_i = 0;
  int16_t roll_target = 1500, pitch_target = 1500;

  #ifdef DEBUG
  char buf[80];
  uint8_t count = 0;
  #endif

  #ifdef DEBUG
  serial_set_baud_rate(115200);
  #endif
  
  servos_start((uint8_t[]){ROLL_SERVO_PIN, PITCH_SERVO_PIN}, 2);

  twi_init();
  
  delay(100);
  
  gyro_init();  
  acc_init();

  delay(20);

  while (1)
  {
    m = millis();
    elapsed = m - last_m;
    
    if(elapsed >= 20)  // Main loop runs at 50Hz
    {
      last_m = m;
      G_Dt = elapsed / 1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    
      // *** DCM algorithm
      // Data acquisition
      gyro_read();
      acc_read();
    
      // Calculations...
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();
      // ***

      roll_last_p = roll_p;
      roll_p = -(roll * 2000 / M_PI);
      roll_d = roll_p - roll_last_p;
      roll_i += roll_p;
      roll_delta = roll_p / 8 + roll_d / 10 + roll_i / 1000;
      roll_target += roll_delta;
      roll_target = constrain(roll_target, 500, 2500);
      
      pitch_last_p = pitch_p;
      pitch_p = -(pitch * 2000 / M_PI);
      pitch_d = pitch_p - pitch_last_p;
      pitch_i += pitch_p;
      pitch_delta = pitch_p / 8 + pitch_d / 10 + pitch_i / 1000;
      pitch_target += pitch_delta;
      pitch_target = constrain(pitch_target, 500, 2500);

      set_servo_target(ROLL_SERVO, roll_target);
      set_servo_target(PITCH_SERVO, pitch_target);
      
      #ifdef DEBUG
      if (++count == 5)
      {
        count = 0;
        sprintf(buf, "p=%i r=%i rd=%i rt=%i pd=%i pt=%i\r\n", (int)(pitch * 180 / M_PI), (int)(roll * 180 / M_PI), roll_delta, roll_target, pitch_delta, pitch_target);
        serial_send(buf, strlen(buf));
      }
      #endif
    }    
  }
}

