#include "src/simple-serial/simpleSerial.h"

//*********************************************************************************************

/* Device version in hex */
#define DEVICE_VERSION (0x000001U)

/* Simple serial titles for different operations */
#define TITLE_INFO        (0x01U)
#define TITLE_STATUS_ALL  (0x02U)
#define TITLE_STATUS_ONE  (0x03U)
#define TITLE_SET_ONE     (0x04U)
#define TITLE_TOGGLE_ONE  (0x05U)

/* Serial port baudrate */
#define SERIAL_BAUDRATE (115200U)

/* Number of channels on the device */
#define NUM_CHANNELS (8U)

/* Macros to define logic levels for relay on state and relay off state */
#define RELAY_ON_STATE HIGH
#define RELAY_OFF_STATE LOW

/* Maximum number of bytes in the response message */
#define RESPONSE_MAX_BYTES (20U)

//*********************************************************************************************





//*********************************************************************************************

typedef uint8_t u8;

typedef struct _rps_command_s_t {
   u8 title;
   void (*handler)(void);
}rps_command_s_t;

//*********************************************************************************************




//*********************************************************************************************

u8 send_msg_buff[RESPONSE_MAX_BYTES];

/* Structures to hold incoming and outgoing simple serial messages */
simple_serial_msg recv_msg;
simple_serial_msg send_msg;

/* This array stores the digital output pins,
these pins will be used to control the relays*/
static u8 channel_map[NUM_CHANNELS] = {
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9
};

/* Array to hold channel states */
static u8 channel_state[NUM_CHANNELS];

static u8 n_rps_commands;

//*********************************************************************************************





//*********************************************************************************************

/* Handler for info command */
void rps_info_handler( void ) {

  /* Prepare the response */
  send_msg.ver = 1;
  send_msg.title = TITLE_INFO;
  send_msg.message[0] = (u8) ( (uint32_t)DEVICE_VERSION >> 16 );
  send_msg.message[1] = (u8) ( (uint32_t)DEVICE_VERSION >> 8 );
  send_msg.message[2] = (u8) ( DEVICE_VERSION );
  send_msg.message[3] = (u8) ( NUM_CHANNELS );

  for( u8 i=0; i<NUM_CHANNELS; i++ ) {
    send_msg.message[ 4 + i ] = (u8) channel_state[ i ];
  }

  send_msg.len = (u8) ( NUM_CHANNELS + 4 );

  /* Send back the response */
  simpleSerial :: send_simple_serial_message( send_msg );
}

/* Handler for status all command */
void rps_status_all_handler( void ) {
  /* Prepare the response */
  send_msg.ver = 1;
  send_msg.title = TITLE_STATUS_ALL;
  send_msg.message[0] = (u8) (NUM_CHANNELS);

  for( u8 i=0; i<NUM_CHANNELS; i++ ) {
    send_msg.message[ 1 + i ] = (u8) channel_state[ i ];
  }

  send_msg.len = (u8) ( NUM_CHANNELS + 1 );

  /* Send back the response */
  simpleSerial :: send_simple_serial_message( send_msg );
}

/* Handler for status one command */
void rps_status_one_handler( void ) {
  
  /* Prepare the response */
  send_msg.ver = 1;
  send_msg.title = TITLE_STATUS_ONE;
  send_msg.message[0] = recv_msg.message[0];
  send_msg.message[1] = (u8) channel_state[ recv_msg.message[0] ];
  send_msg.len = 2;

  /* Send back the response */
  simpleSerial :: send_simple_serial_message( send_msg );
}

/* Handler for set one handler */
void rps_set_one_handler( void ) {

  /* Set the state */
  if( recv_msg.message[1] == 0x00U ) {
    digitalWrite( channel_map[ recv_msg.message[0] ], RELAY_OFF_STATE );
    channel_state[ recv_msg.message[0] ] = (u8) 0x00U;
  }
  else {
    digitalWrite( channel_map[ recv_msg.message[0] ], RELAY_ON_STATE );
    channel_state[ recv_msg.message[0] ] = (u8) 0x01U;
  }
  
  /* Prepare the response */
  send_msg.ver = 1;
  send_msg.title = TITLE_SET_ONE;
  send_msg.len = 0;

  /* Send back the response */
  simpleSerial :: send_simple_serial_message( send_msg );
}

/* This array contains details about different commands that are supported by the rps device
*/
static rps_command_s_t rps_command_map[] = {
  { .title = TITLE_INFO, .handler=&rps_info_handler },
  { .title = TITLE_STATUS_ALL, .handler=&rps_status_all_handler },
  { .title = TITLE_STATUS_ONE, .handler=&rps_status_one_handler },
  { .title = TITLE_SET_ONE, .handler=&rps_set_one_handler }
};

/* This function is used to invoke the right handler for a command */
static u8 rps_decision_tree( void ) {
  
  static u8 inTitle, ret;
  static void (*func)(void);

  inTitle = (recv_msg.title & 0x7FU);
  ret = 0U;
  
  for( static u8 i=0; i<n_rps_commands; i++) {
    
    /* Check whether the command matches*/
    if( inTitle == rps_command_map[i].title ) {
       
       func = rps_command_map[i].handler;
       if( func != NULL ) {
        (*func)();
        ret = 1;
       }
       break;
    }
    
  }

  return ret;
}

//*********************************************************************************************





void setup() {

  n_rps_commands = (sizeof( rps_command_map )/sizeof( rps_command_s_t ));
  send_msg.message = send_msg_buff;
  
  /* Initialize the serial port*/
  Serial.begin( SERIAL_BAUDRATE );

  /* Configure the pin mode */
  for( u8 channel=0; channel<NUM_CHANNELS; channel++ ) {
    pinMode( channel_map[channel], OUTPUT );
    digitalWrite( channel_map[channel], RELAY_OFF_STATE );
    channel_state[ channel ] = (u8) 0x00U;
  }
  
}

void loop() 
{
  /* Feed the incoming bytes in the simple serial state machine */
  simpleSerial :: screen_bytes();

  /* Check if a valid simple serial message is available */
  if( simpleSerial :: simple_serial_available() ) {

    /* Get the simple serial message */
    simpleSerial :: get_simple_serial_message( &recv_msg );

    /* Call the decision tree and send error response if something goes wrong */
    if( rps_decision_tree() == 0U ) {
      send_msg.title = recv_msg.title | (0x80U);
      send_msg.len = 0;

      simpleSerial :: send_simple_serial_message( send_msg );
    }

    /* Free the dynamic memory captured by simple serial */
    simpleSerial :: simple_serial_delete( recv_msg );
  }
}
