#include <pololu/orangutan.h>
#include "twi.h"
#include "sensors.h"
#include "dcm.h"

int main(void)
{
  uint8_t last_update = 0, elapsed;
  
  twi_init();
  
  delay(100);
  
  gyro_init();  
  acc_init();

  delay(20);
  
  while (1)
  {
    elapsed = (uint8_t)millis() - last_update;
    
    if(elapsed >= 20)  // Main loop runs at 50Hz
    {
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
    }    
  }
}

