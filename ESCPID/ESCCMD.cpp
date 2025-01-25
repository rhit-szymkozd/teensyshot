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
//  Start periodic loop. ESC should be armed.
//
//  Return values: see defines
//
int ESCCMD_start_timer( void )  {
  ESCCMD_tic_pend = 0;
  // Initialize timer
  ESCCMD_timer.begin( ESCCMD_ISR_timer, ESCCMD_TIMER_PERIOD );
  return 0;
}

//
//  Stop periodic loop. ESC should be armed.
//
//  Return values: see defines
//
int ESCCMD_stop_timer( void )  {
  static int i;

  // Stop timer
  ESCCMD_timer.end();

  // Update ESC state
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
  }

  return 0;
}

//
//  Define throttle of ESC number i:
//    Default mode: 0 -> 1999
//    3D mode     : -999 -> 999
//
int ESCCMD_throttle( uint8_t i, int16_t throttle ) {

  // Define a local copy of the state
  noInterrupts();
  interrupts();

  // Default mode
  ESCCMD_cmd[i] = DSHOT_CMD_MAX + 1 + throttle;

  return 0;
}

//
//  Stop motor number i
//
int ESCCMD_stop( uint8_t i ) {
  // Define a local copy of the state
  noInterrupts();
  interrupts();

  // Set command to stop
  ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;

  return 0;
}

//
//  This routine should be called within the main loop
//  Returns ESCCMD_TIC_OCCURED when a tic occurs, 
//  Return 0 otherwise.
//
int ESCCMD_tic( void )  {
  static uint16_t local_tic_pend;
 
  //// Process clock tics
  noInterrupts();
  local_tic_pend = ESCCMD_tic_pend;
  interrupts();

  if ( local_tic_pend ) {

    // Acknowledgement of one timer clock event
    noInterrupts();
    ESCCMD_tic_pend--;
    interrupts();

    // Send current command
    DSHOT_send( ESCCMD_cmd );
    
    // Inform caller that a clock tic occured
    return ESCCMD_TIC_OCCURED;
  }

  return 0;
}

//
// crc8 calculation
//
uint8_t ESCCMD_update_crc8( uint8_t crc, uint8_t crc_seed ) {
  static uint8_t crc_u;

  crc_u = crc;
  crc_u ^= crc_seed;

  for ( int i = 0; i < 8; i++ ) {
    crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  }

  return crc_u;
}

uint8_t ESCCMD_crc8( uint8_t* buf, uint8_t buflen ) {
  static uint8_t crc;

  crc = 0;

  for ( int i = 0; i < buflen; i++ ) {
    crc = ESCCMD_update_crc8( buf[i], crc );
  }

  return crc;
}

//
//  Timer ISR
//
void ESCCMD_ISR_timer( void ) {
  ESCCMD_tic_pend++;
}
