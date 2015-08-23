/** OXFloodNet_Display, based on LLAPManager sketch for Ciseco RFu-328 and ST7735 based LCD
 * Using custom PCB with 5 way joystick for navigation
 * Extended for V4 sensors and LLAP Extended protocol.
 *
 * (c) Andrew D Lindsay, Thing Innovations 2015
 */

#define VERSION_STRING "V0.2"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#include "LLAPExtended.h"	// Extende LLAP library
#include "heart.h"        // Heartbeat logo
#include "logo.h"         // OxFloodNet Logo

// Application defines
// Display info
#define LCD_SIZE_X 160
#define LCD_SIZE_Y 128

// Re-define the colours as library is not correct for these displays
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF

// text formatting options
#define TEXT_UNFORMAT 0
#define TEXT_CENTRED 1
#define TEXT_LEFT  2
#define TEXT_RIGHT 3

// joystick number
#define UP_KEY 0
#define DOWN_KEY 1
#define LEFT_KEY 2
#define RIGHT_KEY 3
#define CENTRE_KEY 4

#define JOYSTICK_NONE -1

// Joystick ports for portrait mode
//#define JOYSTICK_UP A0
//#define JOYSTICK_DOWN A1
//#define JOYSTICK_LEFT A2
//#define JOYSTICK_RIGHT A3
//#define JOYSTICK_CENTRE A4

// Joystick ports for Landscape mode, Joystick on Left
//#define JOYSTICK_UP A2
//#define JOYSTICK_DOWN A3
//#define JOYSTICK_LEFT A1
//#define JOYSTICK_RIGHT A0
//#define JOYSTICK_CENTRE A4

// Joystick ports for Landscape mode, Joystick on Right
#define JOYSTICK_UP A3
#define JOYSTICK_DOWN A2
#define JOYSTICK_LEFT A0
#define JOYSTICK_RIGHT A1
#define JOYSTICK_CENTRE A4

// SRF enable
#define SRF_RADIO_ENABLE 8

// LCD Display port definitions
#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  6  // you can also connect this to the Arduino reset

// SD Card enable pin
#define SD_CS 2

// Application specific defines
// Max number of nodes to store data for
#define MAX_NODES 10
// Times for indicating non responsive nodes. Assumes each transmits every 5 mins
//#define TESTING 1
#ifdef TESTING
#define TIME1 60000L
// 20 min
#define TIME2 1200000L
// 30 mins dead time
#define DEAD_TIME 120000L
#else
// 10 min
#define TIME1 600000L
// 20 min
#define TIME2 1200000L
// 30 mins dead time
#define DEAD_TIME 1800000L
#endif

// Display refresh to include non responsive nodes, 60s
//#define DISPLAY_REFRESH 60000L
#define DISPLAY_REFRESH 500L

// Battery reading below which colour changes for low battery
#define BATTERY_THRESHOLD 220

// Range test values
#define NO_RESP_TIME 5000

// Data Structures and global values
// Sensors - Keeps track of display position, last update time
// and battery voltage reading if available
typedef struct Sensors {
  byte x;        // X position on display
  byte y;        // Y position on display
  char msgType;  // a for original LLAP, b for Extended LLAP
  char id[8];    // Node ID
  char rdgType;  // Type of reading, for Extended LLAP
  char msg[16];  // Data plus \0 terminator
  int batt;      // Battery reading * 100, e.g. 250 = 2.50V
  long lastRx;   // Last receive time so can change colour and remove dead nodes
} Sensor;

int lastNodeNum = 0;
long lastUpdate = 0L;
boolean heartbeat = false;

// Node array of readings
Sensor node[MAX_NODES];

// Instantiate the Adafruit ST7735 library. Use SPI interface
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Hardware specific functions
// ***************************
// Joystick
//keypad debounce parameter
#define DEBOUNCE_MAX 15
#define DEBOUNCE_ON  10
#define DEBOUNCE_OFF 3
#define NUM_KEYS 5

#define MENU_X	5		// 0-159
#define MENU_Y	30		// 0-127
#define TEXT_FONT_SIZE 2
#define TEXT_HEIGHT 18
//#define TEXT_FONT_SIZE 1
//#define TEXT_HEIGHT 12

byte joystickPorts[] = {
  JOYSTICK_UP, JOYSTICK_DOWN, JOYSTICK_LEFT, JOYSTICK_RIGHT, JOYSTICK_CENTRE
};
byte joystickKeys[] = {
  UP_KEY, DOWN_KEY, LEFT_KEY, RIGHT_KEY, CENTRE_KEY
};

// debounce counters
byte button_count[NUM_KEYS];
// button status - pressed/released
byte button_status[NUM_KEYS];
// button on flags for user program
byte button_flag[NUM_KEYS];

// Menu setup
#define NUM_MENU_ITEM 3
// Menu text
char menu_items[NUM_MENU_ITEM][15] = {
  "LLAP Display",
  "Range Test",
  "About"
};

// Functions for each menu option
void (*menu_funcs[NUM_MENU_ITEM])(void) = {
  llapDisplay,
  rangeTest,
  about
};

byte current_menu_item;

char *srfErrMsg[6] = { "Done",
                       "Cant set command mode",
                       "ATNT Failed",
                       "ATID Failed",
                       "ATDN Failed"
                     };

// Joystick functions

/** initJoystick - Set all the pint sof handing the joystick
 */
void initJoystick() {
  // Set joystick pint to input with internal pullup
  pinMode( JOYSTICK_UP, INPUT);
  digitalWrite( JOYSTICK_UP, HIGH);
  pinMode( JOYSTICK_DOWN, INPUT);
  digitalWrite( JOYSTICK_DOWN, HIGH);
  pinMode( JOYSTICK_LEFT, INPUT);
  digitalWrite( JOYSTICK_LEFT, HIGH);
  pinMode( JOYSTICK_RIGHT, INPUT);
  digitalWrite( JOYSTICK_RIGHT, HIGH);
  pinMode( JOYSTICK_CENTRE, INPUT);
  digitalWrite( JOYSTICK_CENTRE, HIGH);

  // setup interrupt-driven keypad arrays
  // reset button arrays
  for (byte i = 0; i < NUM_KEYS; i++) {
    button_count[i] = 0;
    button_status[i] = 0;
    button_flag[i] = 0;
  }

  // Setup timer2 -- Prescaler/256
  TCCR2A &= ~((1 << WGM21) | (1 << WGM20));
  TCCR2B &= ~(1 << WGM22);
  TCCR2B = (1 << CS22) | (1 << CS21);

  ASSR |= (0 << AS2);

  // Use normal mode
  TCCR2A = 0;
  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= (0 << OCIE2A);
  TCNT2 = 0x6; // counting starts from 6;
  TIMSK2 = (1 << TOIE2);
  SREG |= 1 << SREG_I;
}


/** updateJoystickKey - ISR to debounce and update joystick positions
 */
void updateJoystickKey() {
  byte i;

  for (i = 0; i < NUM_KEYS; i++)
  {
    if ( digitalRead( joystickPorts[ i ] ) == LOW ) //one key is pressed
    {
      if (button_count[i] < DEBOUNCE_MAX)
      {
        button_count[i]++;
        if (button_count[i] > DEBOUNCE_ON)
        {
          if (button_status[i] == 0)
          {
            button_flag[i] = 1;
            button_status[i] = 1; //button debounced to 'pressed' status
          }
        }
      }
    }
    else // no button pressed
    {
      if (button_count[i] > 0)
      {
        button_flag[i] = 0;
        button_count[i]--;
        if (button_count[i] < DEBOUNCE_OFF) {
          button_status[i] = 0; //button debounced to 'released' status
        }
      }
    }
  }
}


/** getJoystick - Get the current joystick position
 * @return value to indicate which position joystick is in
 */
byte getJoystick() {
  for (int i = 0; i < NUM_KEYS; i++ ) {
    if ( button_flag[i] != 0 ) {
      button_flag[i] = 0; // reset button flag
      return joystickKeys[ i ];
    }
  }
  return JOYSTICK_NONE;
}


/** isJoystick - Quick test if the joystick is in a particular position
 * @return boolean true/false to indicate if joystick is in specified position
 */
boolean isJoystick( byte posn ) {
  return ( digitalRead( posn ) == LOW );
}


/** displayMenuUtem - Display a single menu option, highlight to show
 * it has been selected
 * @param menux - X position
 * @param menuy - Y position
 * @param text - Menu option character pointer, 0 terminated
 * @param highlight - boolean to indicate this is currently selected item
 */
void displayMenuItem(int menux, int menuy, char *text, boolean highlight ) {
  tft.setCursor(menux, menuy);
  tft.setTextSize(TEXT_FONT_SIZE);
  if ( highlight )
    tft.setTextColor(ST7735_BLACK, ST7735_CYAN);
  else
    tft.setTextColor(ST7735_CYAN, ST7735_BLACK);

  tft.setTextWrap(false);
  tft.print(text);
}


/** initMenu - Display all the menu items, highlight first item
 */
void initMenu() {
  for ( int i = 0; i < NUM_MENU_ITEM; i++ ) {
    displayMenuItem(MENU_X, MENU_Y + (i * TEXT_HEIGHT), menu_items[i], (i == 0 ? true : false) );
  }
  current_menu_item = 0;
  showNodeCount();
}


/** toggleHeartbeat - Display the heart and alternate its colour form red to black
 * to make it flash on and off.
 */
void toggleHeartbeat( ) {
  tft.drawBitmap(122, 15, heart, 32, 32, heartbeat ?  ST7735_RED : ST7735_BLACK );
  heartbeat = heartbeat ? false : true;
}


/** displayRSSI - display the current RSSI value
 * @param rssi - pointer to character string representation of RSSI from message
 */
void displayRSSI( char *rssi ) {
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  tft.setCursor(2, 50);
  tft.setTextSize(9);
  tft.print(rssi);
  tft.print("   ");
}

void displayError( uint16_t fCol, uint16_t bCol, char* errMsg ) {
  tft.setTextColor(fCol, bCol);
  tft.println(errMsg);
}

/** rangeTest - The range test menu option. Requires the slave
 * module to be used with this.
 */
void rangeTest() {
  setScreen( (char*)"Range Test" );

  digitalWrite(SRF_RADIO_ENABLE, LOW); // select the radio
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setTextSize(1);
  tft.setCursor(1, 30);
  tft.println("Setting up SRF");

  // Need to set RSSI Master mode
  uint8_t ret = setupSRF();
  tft.setTextSize(1);
  tft.setCursor(1, 30);
  if ( ret > 0 ) {
    displayError(ST7735_RED, ST7735_BLACK, srfErrMsg[ret]);
    delay(10000);
  } else
    tft.println(srfErrMsg[ret]);
  /*
    switch ( ret ) {
      case 1:
        displayError(ST7735_RED, ST7735_BLACK, "Cant set command mode");
        break;
      case 2:
        displayError(ST7735_RED, ST7735_BLACK, "ATNT Failed");
        break;
      case 3:
        displayError(ST7735_RED, ST7735_BLACK, "ATID Failed");
        break;
      case 4:
        displayError(ST7735_RED, ST7735_BLACK, "ATDN Failed");
        break;
      case 5:
        tft.println("Done");
        break;
    }
  */
  digitalWrite(SRF_RADIO_ENABLE, HIGH); // select the radio

  if ( ret > 0 ) {
//    delay(10000);
    return;
  }

  lastUpdate = millis();

  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setTextSize(2);
  tft.setCursor(1, 22);
  tft.println("RSSI Value");

  displayRSSI( "000" );

  // Initialise the LLAPSerial library
  LLAP.init();

  while ( !isJoystick( JOYSTICK_CENTRE ) ) {
    // Wait for message
    if (LLAP.bMsgReceived) {
      char msgArray[15];
      LLAP.sMessage.toCharArray(msgArray, 12 );
      msgArray[12] = '\0';
      char valArray[10];

      if ( LLAP.sMessage.substring(0, 5).equalsIgnoreCase( "RSSIS") ) {
        toggleHeartbeat();
        //String barCountStr = LLAP.sMessage.substring(4,5);
        LLAP.sMessage.substring(6 ).toCharArray( valArray, 4);
        displayRSSI( valArray );

      }

      lastUpdate = millis();
      LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
    }
    if ( lastUpdate + NO_RESP_TIME < millis() ) {
      // No response
      tft.drawBitmap(122, 15, heart, 32, 32, ST7735_BLUE );
    }

    long nowTime = millis();
    while ( nowTime + 500 < millis()) {
      receiveLLAP();
      LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
      nowTime = millis();
    }
    LLAP.SerialEvent();
  }

  tft.setTextSize(1);
  tft.setCursor(1, 30);
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.println("Restoring SRF");

  ret = restoreSRF();

  tft.setTextSize(1);
  tft.setCursor(1, 30);
  if ( ret > 0 ) {
    displayError(ST7735_RED, ST7735_BLACK, srfErrMsg[ret]);
    delay(10000);
  } else
    tft.println(srfErrMsg[ret]);
  /*
    switch ( ret ) {
      case 1:
        displayError(ST7735_RED, ST7735_BLACK, "Cant set command mode");
  //      delay(10000);
        break;
      case 2:
        displayError(ST7735_RED, ST7735_BLACK, "ATNT Failed");
  //      delay(10000);
        break;
      case 3:
        displayError(ST7735_RED, ST7735_BLACK, "ATID Failed");
  //      delay(10000);
        break;
      case 4:
        displayError(ST7735_RED, ST7735_BLACK, "ATDN Failed");
  //      delay(10000);
        break;
      case 5:
        tft.println("Done");
        break;
    }
  */
  //  if ( ret < 5 )
  //    delay(10000);

  digitalWrite(SRF_RADIO_ENABLE, HIGH); // select the radio

  LLAP.init();

}


/** llapDisplay - Main display for received LLAP messages
 * Handles aging of reading so that non functioning nodes will
 * be automatically removed. Only 10 readings are stored.
 * Pressing joystick centre button will exit and return to menu.
 */
void llapDisplay() {
  setScreen( (char*)"LLAP Display" );
  int blankLines = 0;
  lastUpdate = millis();
  while ( !isJoystick( JOYSTICK_CENTRE ) ) {
    if (LLAP.bMsgReceived) {
      displayNode( setNode(LLAP.cMessage) );
      LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
    }
    // Display refresh every minute to show nodes that have dropped out
    long now = millis();
    if ( (lastUpdate + DISPLAY_REFRESH) < now ) {
      int blankLines = deadNodeCheck();
      refreshDisplay( blankLines );
      lastUpdate = millis();
      if ( isJoystick( JOYSTICK_CENTRE ) )
        return;
    }
    LLAP.SerialEvent();
  }
}


/** formatText - format and display text
 * @param x - X position of text
 * @param y - Y position of text
 * @param textSize - Size of text to display. 1-4 with 4 being largest
 * @param format - Lef, right or centred text
 * @param text - pointer to 0 terminated character string to display.
 */
void formatText( int x, int y, int textSize, int format, char *text ) {
  // format text on display
  tft.setTextSize( textSize);
  int pixelsChar = 6;
  switch ( textSize ) {
    case 1:
      pixelsChar = 6;
      break;
    case 2:
      pixelsChar = 12;
      break;
    case 3:
      pixelsChar = 15;
      break;
    case 4:
      pixelsChar = 18;
      break;
  }

  switch ( format ) {
    case TEXT_CENTRED:    // Centered
      x = (LCD_SIZE_X / 2 - ((strlen(text) * pixelsChar) / 2));
      break;
    case TEXT_LEFT:      // Left justified
      x = 0;
      break;
    case TEXT_RIGHT:     // Right Justified
      x = LCD_SIZE_X - (strlen(text) * pixelsChar);
      break;
    default:             // None
      break;
  }

  tft.setCursor(x, y);
  tft.println( text );
}


/** about - Display the about screen and wait for joystick centre
 *  button to be pressed before returning to the menu.
 */
void about() {
  setScreen( (char*)"About" );
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);

  formatText(0, 20, 2, TEXT_CENTRED, "Oxford Flood" );
  formatText(0, 40, 2, TEXT_CENTRED, "Network" );
  formatText(0, 60, 2, TEXT_CENTRED, "LLAP Display" );
  formatText(0, 80, 2, TEXT_CENTRED, VERSION_STRING );
  formatText(0, 100, 1, TEXT_CENTRED, "By Andrew Lindsay" );
  formatText(0, 110, 1, TEXT_CENTRED, "@ThingInnovation" );
  formatText(0, 120, 1, TEXT_CENTRED, "For @oxfloodnet" );

  // Displayed until joystick centre button pressed
  while ( !isJoystick( JOYSTICK_CENTRE ) ) {
    receiveLLAP();
    LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
    LLAP.SerialEvent();
  }
}


/** initNodes - Initialise our node data
 * Set default values for each node element
 */
void initNodes() {
  for (int i = 0; i < MAX_NODES; i++) {
    node[i].x = -1;
    node[i].y = -1;
    node[i].msgType = '-';
    for (int j = 0; j < 8; j++ )
      node[i].id[j] = '-';
    node[i].rdgType = '-';
    node[i].msg[0] = '\0';
    node[i].batt = -1;
    node[i].lastRx = 0L;
  }
  lastNodeNum = 0;
}


/** getNodeId -  Given a node id, locate a node record or -1 if not present
 * Use reading type if using LLAPExtended format
 * @param msgType - The message type, a or b
 * @param id - Pointer to array containing the id
 * @param rdgType - Reading type, used in b only as multiple readings are sent by the sensors
 * @return index for the node or -1 if not found.
 */
int getNodeId( char msgType, char *id, char rdgType ) {
  if ( msgType == 'a' ) {
    for ( int i = 0; i < lastNodeNum; i++ ) {
      if ( id[0] == node[i].id[0] && id[1] == node[i].id[1] )
        return i;
    }
  } else if ( msgType == 'b' ) {
    // Check message type as sensors can send 3 messages
    for ( int i = 0; i < lastNodeNum; i++ ) {
      // TODO: Try strncmp
      if ( id[0] == node[i].id[0] && id[1] == node[i].id[1] &&
           id[2] == node[i].id[2] && id[3] == node[i].id[3] &&
           id[4] == node[i].id[4] && id[5] == node[i].id[5] &&
           id[6] == node[i].id[6] && id[7] == node[i].id[7] &&
           rdgType == node[i].rdgType )
        return i;
    }
  }

  return -1;
}


/** setNode - update stored node data from new receive message
 * Msg is aNNddddddddd (12) or bNNNNNNNNddddddddd (18)
 * Filter out awake and sleep messages.
 * For BATT, only update the battery voltage except for type b as this is a separate message
 * @param msg - pointer to received message
 * @return
 */
int setNode( char *msg ) {
  char id[8] = { '-', '-', '-', '-', '-', '-', '-', '-' };
  char msgType = *msg++;
  char rdgType = '-';

  // msgType a - 2 char id, b - 8 char id
  if ( msgType == 'a' ) {
    id[0] = *msg++;
    id[1] = *msg++;
  } else if ( msgType == 'b' ) {
    for ( int i = 0; i < 8; i++ )
      id[i] = *msg++;
  }

  // Ignore START, SLEEPPING and WAKE messages
  if ( strncmp(msg, "START", 5 ) == 0 ||
       strncmp(msg, "SLEEP", 5 ) == 0 ||
       strncmp(msg, "AWAKE", 5 ) == 0)
    return -1;    // No update

  if ( msgType == 'b' ) {
    rdgType = *msg;
  } else rdgType = '-';

  int nodeNum = getNodeId( msgType, id, rdgType );
  if ( nodeNum == -1 ) {
    if ( (lastNodeNum + 1) >= MAX_NODES )
      return -1;

    nodeNum = lastNodeNum++;

    // Set unchanging values, position and node
    node[nodeNum].x = 2;
    node[nodeNum].y = 15 + (nodeNum * 10);
    node[nodeNum].msgType = msgType;

    // Set Node ID
    if ( msgType == 'a' ) {
      node[nodeNum].id[0] = id[0];
      node[nodeNum].id[1] = id[1];
    } else if ( msgType == 'b' ) {
      for ( int i = 0; i < 8; i++ )
        node[nodeNum].id[i] = id[i];

      node[nodeNum].rdgType = rdgType;
      msg++;    // Skip rdgType
    }

    //    strncpy(node[nodeNum].msg, "---------", 9);
    //    node[nodeNum].msg[9] = '\0';
    node[nodeNum].batt = -1;
    showNodeCount();
  }

  node[nodeNum].lastRx = millis();

  // Set Battery voltage if BATT received
  if ( msgType == 'a' ) {
    if ( strncmp(msg, "BATT", 4 ) == 0 ) {
      node[nodeNum].batt = (int)(atof( &msg[4] ) * 100);
      return -1;
    }
    else {
      // TODO: Strip trailing -
      strncpy( node[nodeNum].msg, msg, 9);
    }
  } else if ( msgType == 'b' ) {
    // Strip trailing -
    //msg++;
    strncpy( node[nodeNum].msg, msg, 9);
    if ( node[nodeNum].msg[8] == '-' ) {
      int count = 8;
      while ( node[nodeNum].msg[count] == '-' && count > 0 ) {
        node[nodeNum].msg[count--] = '\0';
      }
    }

  } else {
    // TODO: Strip trailing -
    strncpy( node[nodeNum].msg, msg, 9);
    if ( node[nodeNum].msg[8] == '-' ) {
      int count = 8;
      while ( node[nodeNum].msg[count] == '-' && count > 0 ) {
        node[nodeNum].msg[count--] = '\0';
      }
    }
  }

  return nodeNum;
}


/** displayNode - Display the details for a single node
 * @param nodeId - The index of the node to display.
 */
void displayNode( int nodeId ) {
  if ( nodeId >= lastNodeNum || nodeId < 0 ||
       nodeId >= MAX_NODES ||
       node[nodeId].x < 0 || node[nodeId].y < 0 )
    return;

  if (node[nodeId].lastRx == 0L )
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  else {
    long now = millis();
    // Oldest are red, then yellow.
    if ((node[nodeId].lastRx + TIME2) < now )
      tft.setTextColor(ST7735_RED, ST7735_BLACK);  // Red
    else if ((node[nodeId].lastRx + TIME1) < now )
      tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);  // Yellow
    else
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  }
  tft.setTextSize(1);

  // TODO: Check why full node ID not shown

  tft.setCursor(node[nodeId].x, node[nodeId].y);
  // Depending on Node type, ID is 2 chars for a, 8 for b

  // next 2 lines just for testing to idenitify node type
  //  tft.write( node[nodeId].msgType);
  //  tft.write( ' ' );

  //Display extended LLAP response
  if ( node[nodeId].msgType == 'a') {
    // Standard LLAP response.
    // Display Node ID
    tft.write(node[nodeId].id[0]);
    tft.write(node[nodeId].id[1]);
    for ( int i = 0; i++; i < 6)
      tft.write(' ');     // Padding

    tft.write(' ');

    // Display reading type and values
    if ( strncmp(node[nodeId].msg, "TMPA", 4 ) == 0 ) {
      tft.print("TMPA ");
      tft.print(&node[nodeId].msg[4]);
      tft.write('C');
    } else if ( strncmp(node[nodeId].msg, "LVAL", 4 ) == 0 ) {
      tft.print("LVAL ");
      tft.print(&node[nodeId].msg[4]);
      tft.write('%');
    } else {
      // Anything else, just display it, e.g. button presses
      tft.print(node[nodeId].msg);
      tft.print("  ");
    }
  } else if ( node[nodeId].msgType == 'b') {
    // Display Node ID
    tft.write(node[nodeId].id[0]);
    tft.write(node[nodeId].id[1]);
    tft.write(node[nodeId].id[2]);
    tft.write(node[nodeId].id[3]);
    tft.write(node[nodeId].id[4]);
    tft.write(node[nodeId].id[5]);
    tft.write(node[nodeId].id[6]);
    tft.write(node[nodeId].id[7]);

    //    for ( int i = 0; i++; i < 8)
    //      tft.write('X');
    //      tft.write(node[nodeId].id[i]);

    tft.write(' ');
    tft.write(node[nodeId].rdgType);
    tft.write(' ');
    tft.print(&node[nodeId].msg[0]);  // Should just be value
  }
  tft.write( ' ' );

  // If there is a battery reading show it.
  if ( node[nodeId].batt >= 0 ) {
    if ( node[nodeId].batt >= BATTERY_THRESHOLD ) {
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    }
    else {
      tft.setTextColor(ST7735_RED, ST7735_BLACK); // Red
    }
    tft.print(node[nodeId].batt / 100);
    tft.write( '.' );
    tft.print(node[nodeId].batt % 100);
    tft.write( 'V' );
  }
  else {
    // Display blank for voltage
    tft.print("     ");
  }
}


/** showNodeCount - Display status line on display
 * showing node count and battery voltage.
 */
void showNodeCount( ) {
  tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, LCD_SIZE_Y - 10 );
  tft.print("Nodes:");
  tft.print(lastNodeNum );
  tft.write(' ');

  // Show battery voltage
  long mV = readVcc();
  tft.setCursor(LCD_SIZE_X - 60, LCD_SIZE_Y - 10 );
  tft.print("Batt:");
  tft.print(mV / 1000);
  tft.print(".");
  int mod = (mV % 1000) / 10;
  if ( mod < 10 )
    tft.print("0");
  tft.print((mV % 1000) / 10);
  tft.print("V");
}


/** refreshDisplay - Redisplay all the node information
 * @param blankLines - current number of blank lines
 */
void refreshDisplay( int blankLines ) {
  for ( int i = 0; i < lastNodeNum; i++ ) {
    displayNode( i );
  }
  if ( blankLines > 0 ) {
    // Get last Y
    byte yPos = 15 + (10 * lastNodeNum);
    for ( int i = 0; i < blankLines; i ++ ) {
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      tft.setTextSize(1);
      tft.setCursor(0, yPos);
      tft.print("                     ");
      yPos += 10;
    }
  }
  showNodeCount();
}


/** removeNode - Remove a node from the list of nodes
 * as it is no longer sending data.
 * @param nodeNum - The index of the node to remove
 */
void removeNode( int nodeNum ) {
  if ( nodeNum >= lastNodeNum || nodeNum < 0 )
    return;

  for (int i = nodeNum; i < (lastNodeNum - 1); i++) {
    node[i].x = node[i + 1].x ;

    for (int j = 0; i < 8; i++ )
      node[i].id[j] = node[i + 1].id[j];

    node[i].msgType = node[i + 1].msgType;
    node[i].rdgType = node[i + 1].rdgType;
    strncpy(node[i].msg, node[i + 1].msg, 10);
    node[i].batt = node[i + 1].batt;
    node[i].lastRx = node[i + 1].lastRx;
  }
  node[lastNodeNum].x = -1;
  node[lastNodeNum].y = -1;
  for (int i = 0; i < 8; i++ )
    node[lastNodeNum].id[i] = '-';

  node[lastNodeNum].msg[0] = '\0';
  node[lastNodeNum].batt = -1;
  node[lastNodeNum].lastRx = 0L;

  lastNodeNum--;
}


/** deadNodeCheck - Check that nodes are still responding
 */
int deadNodeCheck() {
  int deadNodes = 0;
  for ( int i = 0; i < lastNodeNum; ) {
    long now = millis();
    if ((node[i].lastRx + DEAD_TIME) < now ) {
      // Remove node
      removeNode( i );
      deadNodes++;
    }
    else
      i++;
  }
  return deadNodes;
}


/** receiveLLAP - check for received message and find node data
 * @return Node index of the message just received.
 */
int receiveLLAP() {
  if (LLAP.bMsgReceived) {
    int n = setNode(LLAP.cMessage);
    //    LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
    return n;
  }
  return -1;
}


/* setupSRF - Setup SRF for range testing
 */
uint8_t setupSRF()
{
  if (!enterCommandMode())	// if failed once then try again
  {
    if (!enterCommandMode()) return 1;
  }
  if (!sendCommand("ATNT3")) return 2;
  if (!sendCommand("ATID2305")) return 3;
  if (!sendCommand("ATDN")) return 4;
  return 0; // success
}


/** restoreSRF - Return SRF to normal operation.
 */
uint8_t restoreSRF()
{
  if (!enterCommandMode())	// if failed once then try again
  {
    if (!enterCommandMode()) return 1;
  }
  if (!sendCommand("ATNT0")) return 2;
  if (!sendCommand("ATID5AA5")) return 3;
  if (!sendCommand("ATDN")) return 4;
  return 0; // success
}


/** enterCommandMode - Send +++ to enter command mode on SRF in order to
 * be able to send AT commands.
 * @return 1 for successfully entered command mode
 */
uint8_t enterCommandMode()
{
  delay(1200);
  Serial.print(F("+++"));
  delay(500);
  while (Serial.available()) Serial.read();  // flush serial in - get rid of anything received before the +++ was accepted
  delay(500);
  return checkOK(500);
}


/** sendCommand - send an AT command to the SRF.
 * @param lpszCommand - char pointer to AT command to send
 * @return 1 for OK received and command executed.
 */
uint8_t sendCommand(char* lpszCommand)
{
  Serial.print(lpszCommand);
  Serial.write('\r');
  return checkOK(100);
}

/** checkOK - check for receive OK after sending AT command
 * @param timeout - time in mS to wait for a response
 * @return 1 for OK recieved in time and 0 for timeout
 */
uint8_t checkOK(int timeout)
{
  uint32_t time = millis();
  while (millis() - time < timeout)
  {
    if (Serial.available() >= 3)
    {
      if (Serial.read() != 'O') continue;
      if (Serial.read() != 'K') continue;
      if (Serial.read() != '\r') continue;
      return 1;
    }
  }
  return 0;
}


/** setScreen - Clear screen and add title
 */
void setScreen( char *title ) {
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_CYAN, ST7735_BLACK);
  formatText(0, 0, 2, TEXT_CENTRED, title );
}


/** readVcc - Battery monitoring using internal analog to digital convertor
 * to read battery voltage.
 * @return voltage in mV
 */
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


/** setup - Initialise hardware and data
 */
void setup() {
  // Initialise hardware
  initJoystick();
  pinMode(TFT_CS, OUTPUT);
  digitalWrite( TFT_CS, HIGH );

  // Force SD Card to be disabled
  pinMode( SD_CS, OUTPUT );
  digitalWrite( SD_CS, HIGH );

  // Initialise Data structures
  initNodes();
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // If your TFT's plastic wrap has a Red Tab, use the following:
  //tft.initR(INITR_REDTAB);   // initialize a ST7735R chip, red tab
  // If your TFT's plastic wrap has a Green Tab, use the following:
  //tft.initR(INITR_GREENTAB); // initialize a ST7735R chip, green tab
  //  tft.fillScreen(ST7735_BLACK);

  tft.setTextWrap(false);

  // Rotate screen so I can use bigger characters
  //  tft.setRotation(3);
  tft.setRotation(1);
  tft.fillScreen(ST7735_WHITE);

  // Splash screen
  tft.drawBitmap(16, 0, OxFloodNet, 128, 128, ST7735_BLACK );
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  formatText(0, 2, 2, TEXT_CENTRED, "Oxford Flood" );
  formatText(0, 110, 2, TEXT_CENTRED, "Network" );

  delay(5000);
  tft.fillScreen(ST7735_BLACK);

  setScreen((char*)"OXFloodNet");

  // initialise serial:
  Serial.begin(115200);
  pinMode(SRF_RADIO_ENABLE, OUTPUT);    // initialize pin 8 to control the radio
  digitalWrite(SRF_RADIO_ENABLE, HIGH); // select the radio

  // Initialise the LLAPSerial library
  LLAP.init();

  initMenu();

  lastUpdate = millis();
}


/** loop - main processing, handle joystick input, menu selection,
 * receive messages
 */
void loop() {

  byte i;
  for (i = 0; i < NUM_KEYS; i++) {
    if (button_flag[i] != 0) {

      button_flag[i] = 0; // reset button flag
      switch (i) {
        case UP_KEY:
          // current item to normal display
          displayMenuItem(MENU_X, MENU_Y + (current_menu_item * TEXT_HEIGHT), menu_items[current_menu_item], false );
          current_menu_item -= 1;
          if (current_menu_item < 0)  current_menu_item = NUM_MENU_ITEM - 1;
          // next item to highlight display
          displayMenuItem(MENU_X, MENU_Y + (current_menu_item * TEXT_HEIGHT), menu_items[current_menu_item], true );
          break;
        case DOWN_KEY:
          // current item to normal display
          displayMenuItem(MENU_X, MENU_Y + (current_menu_item * TEXT_HEIGHT), menu_items[current_menu_item], false );
          current_menu_item += 1;
          if (current_menu_item > (NUM_MENU_ITEM - 1))  current_menu_item = 0;
          // next item to highlight display
          displayMenuItem(MENU_X, MENU_Y + (current_menu_item * TEXT_HEIGHT), menu_items[current_menu_item], true );
          break;
        case LEFT_KEY:
          initMenu();
          current_menu_item = 0;
          break;
        case RIGHT_KEY:
          (*menu_funcs[current_menu_item])();
          setScreen( (char*)"OxFloodNet" );
          initMenu();
          current_menu_item = 0;
          break;
      }
    }
  }
  receiveLLAP();
  LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
}


// Timer2 interrupt routine -
// 1/(160000000/256/(256-6)) = 4ms interval

ISR(TIMER2_OVF_vect) {
  TCNT2  = 6;
  updateJoystickKey();
}
// That's all folks

