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
#include "ESCCMD.h"


// Defines
#define ESCPID_NB_ESC             1                 // Number of ESCs
#define ESCPID_MAX_ESC            6                 // Max number of ESCs

#define ESCPID_USB_UART_SPEED     115200            // Baudrate of the teeensy USB serial link

//
//  Arduino setup function
//
void setup() {
  int i;

  // Initialize USB serial link
  Serial.begin( ESCPID_USB_UART_SPEED );

  // Initialize the CMD subsystem
  ESCCMD_init( ESCPID_NB_ESC );

  // Arming ESCs
  ESCCMD_arm_all( );
  
  // Stop all motors
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCCMD_stop( i );
  }
}

//
//  Arduino main loop
//
void loop( ) {
  uint16_t ESCCMD_cmd[] = {148, 0, 0, 0, 0, 0};
  DSHOT_send( ESCCMD_cmd );
}
