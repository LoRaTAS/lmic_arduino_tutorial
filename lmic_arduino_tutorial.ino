/*
  File: lmic_arduino_tutorial.ino
  Author: Stephen Hellicar <stephen.hellicar@data61.csiro.au>

  Description: Basic tutorial covering LMIC library usage
  Modified LMIC library will print when it's transmitting data
  If it fails to join, it will keep trying to join until successful which can take a while
*/

// required includes for LMIC
#include <lmic.h>
#include <hal/hal.h>

// forward declaration of our own handler functions
void OnTransferComplete();
void OnJoinComplete();

// EUI-64 for this Device
// adapted from MAC address
// ie: 0x9876B6106A4E becomes 0x9876B6 0xFFFE 0x106A4E
// refer to your devices MAC address
static const u1_t DEVEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// EUI - Extended Unique Identifiers
// EUI-64 for the Application
static const u1_t APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Application Key (128-bits)
static const u1_t APPKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


// these functions are declared in the LMIC library and must be implemented, they copy the keys into a buffer
// App EUI
void os_getArtEui(u1_t *buf)
{
  memcpy(buf, APPEUI, 8);
}
// Dev EUI
void os_getDevEui(u1_t *buf)
{
  memcpy(buf, DEVEUI, 8);
}
// App Key
void os_getDevKey(u1_t *buf)
{
  memcpy(buf, APPKEY, 16);
}

// pin map for LMIC functionality
// see https://github.com/matthijskooijman/arduino-lmic#dio-pins
// For the Feather M0, DIO0 is pin 3 (https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/pinouts#rfm-slash-semtech-radio-module)
// For the Feather 32u4, DIO0 is pin 7 (https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/pinouts#rfm-slash-semtech-radio-module)
// The other pins are also specified there
// DIO1 is must be connected to another pin via soldered wire and specified
const int DIO0_PIN = 3;
const int DIO1_PIN = 6;
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {DIO0_PIN, DIO1_PIN, LMIC_UNUSED_PIN},
};

#define PRINT_EVENTS

// debug code for printing events, not required
#ifdef PRINT_EVENTS
const int EventCount = 15;
const char *EventName[EventCount] = {
    "Scan Timeout",
    "Beacon Found",
    "Beacon Missed",
    "Beacon Tracked",
    "Joining",
    "Joined",
    "RFU",
    "Join Failed",
    "Rejoin Failed",
    "TX Complete",
    "Lost TSync",
    "Reset",
    "RXComplete",
    "Link Dead"
    "Link Alive"};
#endif

// declared in LMIC library, handles events
// for more info on handling events
// refer to https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-otaa/ttn-otaa.ino
void onEvent(ev_t ev)
{
#ifdef PRINT_EVENTS
  Serial.print("Event: ");
  if (ev >= 1 && ev <= EventCount)
    Serial.println(EventName[ev - 1]);
#endif

  // handle events here
  // we declare functions to handle specific events and dispatch to the correct one
  switch (ev)
  {
  // transfer is completed
  case EV_TXCOMPLETE:
    OnTransferComplete();
    break;

  // joined successfully
  case EV_JOINED:
    OnJoinComplete();
    break;

  case EV_JOINING:
    // do nothing
    break;

  default:
    Serial.println("unhandled event");
    break;
  }
}

// function to setup & initialise LMIC
void setup_lmic()
{
  Serial.println("Setting up LMIC");
  os_init();
  LMIC_reset();
  LMIC_startJoining();
}

// structure for holding our TDF data, must match up with TDF specification
struct GPS_TDF_DATA
{
  GPS_TDF_DATA()
  {
    memset(this, 0, sizeof(GPS_TDF_DATA));
  }

  int32_t lon;       // cast:(double)x*1E-7          %.7f
  int32_t lat;       // cast:(double)x*1E-7          %.7f
  uint32_t height;   // x*0.001                      %.3f m
  uint16_t accuracy; // x*0.001                      %.3f m
  uint16_t heading;  // (x <<10)*1E-5                %.3f
  uint16_t speed;    // x*0.01                       %.2f m/s
  uint8_t pdop;      // (x << 3)*0.01              %.1f
  uint8_t fix;       // ? not sure what this means   %u
  uint8_t numsv;     //                              %u
  uint8_t flags;     // ? not sure what this means   %02x
}
// pack the data as tighly as possible
__attribute__((packed));

// structure for the TDF data + SID header
struct GPS_TDF
{
  // SID as specified in TDF
  const int16_t GPS_SID = 281L;
  // the actual data
  GPS_TDF_DATA Data;
}
// pack the data as tighly as possible
__attribute__((packed));

// send raw binary data of specified length
void sendRawData(const unsigned char *data, uint8_t len, bool confirm)
{
  // quick check so we don't overwrite pending data
  if (LMIC.opmode & OP_TXRXPEND)
    return;

  LMIC_setTxData2(1, (xref2u1_t)data, len, confirm);
  Serial.print("Sending ");
  Serial.print(len);
  Serial.println(" bytes of data");
}

// send our TDF data
void sendTDFData(const struct GPS_TDF &tdf, bool confirm)
{
  const unsigned char *data = reinterpret_cast<const unsigned char *>(&tdf);
  sendRawData(data, sizeof(tdf), confirm);
}

// our function to send data
void DoSendData()
{
  GPS_TDF tdf;
  // send dummy data
  tdf.Data.lat = 1234567890;

  sendTDFData(tdf, true);
}

// called when a transfer is completed
void OnTransferComplete()
{
  Serial.println("Transfer Complete");
  
  // these flags will be set if we try to send confirmed data
  if (LMIC.txrxFlags & TXRX_ACK)
    Serial.println("Received ACK");
  else if (LMIC.txrxFlags & TXRX_NACK)
    Serial.println("No ACK received");

  // wait 5 seconds then send data
  delay(5000);
  DoSendData();
}

// called when we join the network
void OnJoinComplete()
{
  Serial.println("Joined network");
  // send data immediately
  DoSendData();
}

// Arduino setup function
void setup()
{
  // open serial
  Serial.begin(9600);

  // wait until USB is connected
  // useful for debugging
  while (!Serial)
    delay(1);

  Serial.println("Started program");

  // do setup
  setup_lmic();
}

// Arduino loop function
void loop()
{
  // must call LMIC update function
  os_runloop_once();
}
