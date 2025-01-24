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

// Error handling
#define ESCCMD_ERROR( code )              { ESCCMD_last_error[i] = code; return code; }

//
//  Global variables
//
volatile uint8_t    ESCCMD_n;                               // Number of initialized outputs

volatile uint8_t    ESCCMD_state[ESCCMD_MAX_ESC];           // Current state of the cmd subsystem
uint16_t            ESCCMD_CRC_errors[ESCCMD_MAX_ESC];      // Overall number of CRC error since start
int8_t              ESCCMD_last_error[ESCCMD_MAX_ESC];      // Last error code
uint16_t            ESCCMD_cmd[ESCCMD_MAX_ESC];             // Last command
uint16_t            ESCCMD_throttle_wd[ESCCMD_MAX_ESC];     // Throttle watchdog counter
uint8_t             ESCCMD_tlm_deg[ESCCMD_MAX_ESC];         // ESC temperature (°C)
uint16_t            ESCCMD_tlm_volt[ESCCMD_MAX_ESC];        // Voltage of the ESC power supply (0.01V)
uint16_t            ESCCMD_tlm_amp[ESCCMD_MAX_ESC];         // ESC current (0.01A)
uint16_t            ESCCMD_tlm_mah[ESCCMD_MAX_ESC];         // ESC consumption (mAh)
uint16_t            ESCCMD_tlm_rpm[ESCCMD_MAX_ESC];         // ESC electrical rpm (100rpm)
uint8_t             ESCCMD_tlm[ESCCMD_MAX_ESC];             // Set to 1 when asking for telemetry
uint8_t             ESCCMD_tlm_pend[ESCCMD_MAX_ESC];        // Flag indicating a pending telemetry data request
uint8_t             ESCCMD_tlm_valid[ESCCMD_MAX_ESC];       // Flag indicating the validity of telemetry data
uint8_t             ESCCMD_tlm_lost_cnt[ESCCMD_MAX_ESC];    // Lost packet counter of telemetry data
uint64_t            ESCCMD_tic_counter = 0;                 // Counts the number of clock iterations

volatile uint16_t   ESCCMD_tic_pend = 0;                    // Number of timer tic waiting for ackowledgement
volatile uint8_t    ESCCMD_init_flag = 0;                   // Subsystem initialization flag
volatile uint8_t    ESCCMD_timer_flag = 0;                  // Periodic loop enable/disable flag

IntervalTimer       ESCCMD_timer;                           // Timer object
uint8_t             ESCCMD_bufferTlm[ESCCMD_NB_UART][ESCCMD_TLM_LENGTH];

//
//  Initialization
//
void ESCCMD_init( uint8_t n )  {
  static int i;
  ESCCMD_n = n;
  // Initialize data arrays to zero
  for ( i = 0; i < ESCCMD_n; i++ ) {
    ESCCMD_state[i]       = 0;
    ESCCMD_CRC_errors[i]  = 0;
    ESCCMD_last_error[i]  = 0;
    ESCCMD_cmd[i]         = 0;
    ESCCMD_throttle_wd[i] = 0;
    ESCCMD_tlm_deg[i]     = 0;
    ESCCMD_tlm_volt[i]    = 0;
    ESCCMD_tlm_amp[i]     = 0;
    ESCCMD_tlm_mah[i]     = 0;
    ESCCMD_tlm_rpm[i]     = 0;
    ESCCMD_tlm[i]         = 0;
    ESCCMD_tlm_pend[i]    = 0;
    ESCCMD_tlm_valid[i]   = 0;
    ESCCMD_tlm_lost_cnt[i]= 0;
  }

  // Initialize DSHOT generation subsystem
  DSHOT_init( ESCCMD_n );

  // Set the initialization flag
  ESCCMD_init_flag = 1;
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
    ESCCMD_tlm[i] = 0;
  }

  // Send command ESCCMD_CMD_ARMING_REP times
  for ( i = 0; i < ESCCMD_CMD_ARMING_REP; i++ )  {

    // Send DSHOT signal to all ESCs
    DSHOT_send( ESCCMD_cmd, ESCCMD_tlm );

    // Wait some time
    delayMicroseconds( 2 * ESCCMD_CMD_DELAY );
  }

  // Set the arming flag
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] |= ESCCMD_STATE_ARMED;

  return 0;
}

//
//  Activate 3D mode
//
//  Return values: see defines
//
int ESCCMD_3D_on( void )  {
  static int i;

  // Define 3D on command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_3D_MODE_ON;
    ESCCMD_tlm[i] = 1;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    DSHOT_send( ESCCMD_cmd, ESCCMD_tlm );

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Define save settings command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_SAVE_SETTINGS;
    ESCCMD_tlm[i] = 1;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    DSHOT_send( ESCCMD_cmd, ESCCMD_tlm );

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Set the 3D mode flag
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] |= ESCCMD_STATE_3D;

  // Minimum delay before next command
  delayMicroseconds( ESCCMD_CMD_SAVE_DELAY );

  // ESC is disarmed after previous delay
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] &= ~(ESCCMD_STATE_ARMED);

  return 0;
}

//
//  Deactivate 3D mode
//
//  Return values: see defines
//
int ESCCMD_3D_off( void )  {
  static int i;

  // Define 3D off command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_3D_MODE_OFF;
    ESCCMD_tlm[i] = 1;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    DSHOT_send( ESCCMD_cmd, ESCCMD_tlm );

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Define save settings command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_SAVE_SETTINGS;
    ESCCMD_tlm[i] = 1;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    DSHOT_send( ESCCMD_cmd, ESCCMD_tlm );

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Clear the 3D mode flag
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] &= ~(ESCCMD_STATE_3D);

  // Minimum delay before next command
  delayMicroseconds( ESCCMD_CMD_SAVE_DELAY );

  // ESC is disarmed after previous delay
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] &= ~(ESCCMD_STATE_ARMED);

  return 0;
}

//
//  Start periodic loop. ESC should be armed.
//
//  Return values: see defines
//
int ESCCMD_start_timer( void )  {
  static int i;

  // Initialize ESC structure and clear UART input buffer
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
    ESCCMD_tlm[i] = 0;
    ESCCMD_tlm_pend[i] = 0;
    ESCCMD_tlm_lost_cnt[i] = 0;
    ESCCMD_CRC_errors[i] = 0;
    ESCCMD_last_error[i]  = 0;
    ESCCMD_throttle_wd[i] = ESCCMD_THWD_LEVEL;
  }

  ESCCMD_tic_pend = 0;

  // Initialize timer
  ESCCMD_timer.begin( ESCCMD_ISR_timer, ESCCMD_TIMER_PERIOD );
  
  // Raise the timer flag
  ESCCMD_timer_flag = 1;

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
  ESCCMD_timer_flag = 0;

  // Update ESC state
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
    ESCCMD_tlm[i] = 0;
    ESCCMD_state[i] &= ~( ESCCMD_STATE_ARMED | ESCCMD_STATE_START );
  }

  return 0;
}

//
//  Define throttle of ESC number i:
//    Default mode: 0 -> 1999
//    3D mode     : -999 -> 999
//
int ESCCMD_throttle( uint8_t i, int16_t throttle ) {
  static uint8_t local_state;

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Define throttle depending on the mode
  if ( local_state & ESCCMD_STATE_3D )  {
    // 3D mode
    // 48 - 1047    : positive direction (48 slowest)
    // 1048 - 2047  : negative direction (1048 slowest)
    if ( throttle >= 0 )
      ESCCMD_cmd[i] = DSHOT_CMD_MAX + 1 + throttle;
    else
      ESCCMD_cmd[i] = DSHOT_CMD_MAX + 1 + ESCCMD_MAX_3D_THROTTLE - throttle;
  }
  else {
    // Default mode
    ESCCMD_cmd[i] = DSHOT_CMD_MAX + 1 + throttle;
  }

  // Switch start mode on only if needed
  if ( !( local_state & ESCCMD_STATE_START ) ) {
    noInterrupts();
    ESCCMD_state[i] |= ESCCMD_STATE_START;
    interrupts();
    ESCCMD_tlm[i] = 1;
  }

  // Reset the throttle watchdog
  if ( ESCCMD_throttle_wd[i] >= ESCCMD_THWD_LEVEL ) {
    // If watchdog was previously triggered:
    //  Clear UART input buffer
    //  Also clear pending errors, pending packets...
    ESCCMD_tlm_pend[i] = 0;
    ESCCMD_tlm_lost_cnt[i] = 0;
    ESCCMD_CRC_errors[i] = 0;
    ESCCMD_last_error[i]  = 0;
  }
  ESCCMD_throttle_wd[i] = 0;

  return 0;
}

//
//  Stop motor number i
//
int ESCCMD_stop( uint8_t i ) {
  static uint8_t local_state;

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Set command to stop
  ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;

  // Switch start mode off only if needed
  if ( local_state & ESCCMD_STATE_START  ) {
    noInterrupts();
    ESCCMD_state[i] &= ~ESCCMD_STATE_START;
    interrupts();
    ESCCMD_tlm[i] = 0;
  }

  return 0;
}

//
//  This routine should be called within the main loop
//  Returns ESCCMD_TIC_OCCURED when a tic occurs, 
//  Return 0 otherwise.
//
int ESCCMD_tic( void )  {
  static int      i;
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

    // Update counters
    ESCCMD_tic_counter++;
    
    if ( !( ESCCMD_tic_counter % ESCCMD_TLM_PER ) ) {
      
      // Clear stat counters every ESCCMD_TLM_PER iterations
      for ( i = 0; i < ESCCMD_n; i++ )  {
        ESCCMD_tlm_lost_cnt[i] = 0;
        ESCCMD_CRC_errors[i] = 0; 
      }
    }

    // Check if everything is initialized
    if ( !ESCCMD_init_flag )  {
      for ( i = 0; i < ESCCMD_n; i++ )  {
        ESCCMD_last_error[i] = ESCCMD_ERROR_INIT;
      }
      return ESCCMD_TIC_OCCURED;
    }

    // Check if all ESC are armed
    for ( i = 0; i < ESCCMD_n; i++ )  {
      if ( !( ESCCMD_state[i] & ESCCMD_STATE_ARMED ) )  {
        ESCCMD_last_error[i] = ESCCMD_ERROR_SEQ;
        return ESCCMD_TIC_OCCURED;
      }
    }

    // Throttle watchdog
    for ( i = 0; i < ESCCMD_n; i++ )  {
      if ( ESCCMD_throttle_wd[i] >= ESCCMD_THWD_LEVEL )  {
        // Watchdog triggered on ESC number i
        ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
        ESCCMD_tlm[i] = 0;
        ESCCMD_last_error[i] = 0;
        ESCCMD_tlm_lost_cnt[i] = 0;
        ESCCMD_CRC_errors[i] = 0;
        noInterrupts();
        ESCCMD_state[i] &= ~( ESCCMD_STATE_START );
        interrupts();
      }
      else {
        ESCCMD_throttle_wd[i]++;
      }
    }

    // Send current command
    DSHOT_send( ESCCMD_cmd, ESCCMD_tlm );
    
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
  static int i;

  // Check for maximum missed tics (ESC watchdog timer = 250ms on a KISS ESC)
  if ( ESCCMD_tic_pend >= ESCCMD_TIMER_MAX_MISS )  {

    // ESC watchdog switch to disarmed and stopped mode
    for ( i = 0; i < ESCCMD_n; i++ )  {
      ESCCMD_state[i] &= ~( ESCCMD_STATE_ARMED | ESCCMD_STATE_START );
    }
  }
  else  {
    ESCCMD_tic_pend++;
  }
}
