/*
 *  ESCCMD:   ESC DSHOT command packets formating API
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

//
//  Global variables
//
volatile uint8_t    ESCCMD_n;                               // Number of initialized outputs

uint16_t            ESCCMD_CRC_errors[ESCCMD_MAX_ESC];      // Overall number of CRC error since start
uint16_t            ESCCMD_cmd[ESCCMD_MAX_ESC];             // Last command

volatile uint16_t   ESCCMD_tic_pend = 0;                    // Number of timer tic waiting for ackowledgement

IntervalTimer       ESCCMD_timer;                           // Timer object

//
//  Initialization
//
void ESCCMD_init( uint8_t n )  {
  static int i;
  ESCCMD_n = n;
  // Initialize data arrays to zero
  for ( i = 0; i < ESCCMD_n; i++ ) {
    ESCCMD_cmd[i]         = 0;
  }

  // Initialize DSHOT generation subsystem
  DSHOT_init( ESCCMD_n );
}

//
//  Arm all ESCs
//
//  Return values: see defines
//
int ESCCMD_arm_all( void )  {
  static int i;

  // Define stop command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
  }

  // Send command ESCCMD_CMD_ARMING_REP times
  for ( i = 0; i < ESCCMD_CMD_ARMING_REP; i++ )  {

    // Send DSHOT signal to all ESCs
    DSHOT_send( ESCCMD_cmd );

    // Wait some time
    delayMicroseconds( 2 * ESCCMD_CMD_DELAY );
  }
  return 0;
}

//
//  Define throttle of ESC number i:
//    Default mode: 0 -> 1999
//    3D mode     : -999 -> 999
//
int ESCCMD_throttle( uint8_t i, int16_t throttle ) {
  // Default mode
  ESCCMD_cmd[i] = DSHOT_CMD_MAX + 1 + throttle;
  return 0;
}

//
//  Stop motor number i
//
int ESCCMD_stop( uint8_t i ) {
  // Set command to stop
  ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;

  return 0;
}