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
#include "ESCPID.h"

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
  
  // Start periodic loop
  ESCCMD_start_timer( );
  
  // Stop all motors
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCCMD_stop( i );
  }
}

//
//  Arduino main loop
//
void loop( ) {
  if ( ESCCMD_tic( ) == ESCCMD_TIC_OCCURED )  {
    ESCCMD_throttle( 0, (int16_t)100 );
  }
}
