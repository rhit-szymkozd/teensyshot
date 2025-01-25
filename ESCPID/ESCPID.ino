/*
 *  ESCPID:   PID control of up to 6 ESCs using teensy 3.5 MCU
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

// Includes
#include <Arduino.h>
#include "DSHOT.h"

// Defines
#define ESCPID_NB_ESC             1                 // Number of ESCs
#define ESCPID_USB_UART_SPEED     115200            // Baudrate of the teeensy USB serial link

uint16_t cmd[] = {0, 0, 0, 0, 0, 0};

//
//  Arduino setup function
//
void setup() {
  Serial.begin( ESCPID_USB_UART_SPEED );

  // Initialize DSHOT generation subsystem
  DSHOT_init( ESCPID_NB_ESC );

  DSHOT_send( cmd );
  delay(5000);
}

//
//  Arduino main loop
//
void loop( ) {
  // fast sine wave
  for (double i = 0; i <= 2*PI; i = i + 0.01) {
    cmd[0] = (1 - cos(i))*1000/2 + 48;
    Serial.println(cmd[0]);
    DSHOT_send( cmd );
  }
  // // on/off 2s period
  // cmd[0] = 2000;
  // DSHOT_send( cmd );
  // delay(2000);
  // cmd[0] = 0;
  // DSHOT_send( cmd );
  // delay(2000);
}