// 13 Feb 2019 - IST 11:30 pm - D.Lahiri
// Include the GSM library
#include <MKRGSM.h>
// Include the Timer library
#include <Adafruit_ZeroTimer.h>
// Include Watchdog library
#include <Sodaq_wdt.h>

// Tala Code to compare
#define NUM_OF_TALA_CODE_BYTES (3u)
unsigned char talaCode[NUM_OF_TALA_CODE_BYTES] = {0x80, 0x00, 0x00};
// PIN Number
//const char PINNUMBER[] = SECRET_PINNUMBER;
#define PINNUMBER "1413"

//#define TALA_DIRECT
#define TALA_VIA_SLICER

#define TALA_PROTO_SMS_USA_PHONE_NUMBER //  Somnathda 
//#define TALA_PROTO_SMS_USA_TEST_PHONE_NUMBER // Now Victor - change
//#define TALA_PROTO_SMS_INDIA_PHONE_NUMBER // Debojyoti Lahiri

// Phone numbers and messages
#ifdef TALA_PROTO_SMS_USA_PHONE_NUMBER
#define PHONE_NUM_1 "+14084203909" // Number of Dr Somnath Mmukherjee
String gPhoneNumber1 = PHONE_NUM_1;
#define SMS_TXT_1 "Hi Somnath! Intrusion Detected !!! " // Text to Somnath Mmukherjee
String gTextForChabiAlarm1 = SMS_TXT_1;
#endif // #ifdef TALA_PROTO_SMS_USA_PHONE_NUMBER

#ifdef TALA_PROTO_SMS_USA_TEST_PHONE_NUMBER
#define PHONE_NUM_3 "+13038189093" // Number of Victor
String gPhoneNumber3 = PHONE_NUM_3;
#define SMS_TXT_3 "Hi Victor! Intrusion Detected !!! ** Just for demo from Somnath, Please Ignore & delete **" // Text to Debojyoti Lahiri
String gTextForChabiAlarm3 = SMS_TXT_3;
#endif //#ifdef TALA_PROTO_SMS_USA_TEST_PHONE_NUMBER

#ifdef TALA_PROTO_SMS_INDIA_PHONE_NUMBER
#define PHONE_NUM_2 "+919830724896" // Number of Mr Debojyoti Lahiri
String gPhoneNumber2 = PHONE_NUM_2;
#define SMS_TXT_2 "Hi Debojyoti! Intrusion Detected !!! " // Text to Debojyoti Lahiri
String gTextForChabiAlarm2 = SMS_TXT_2;
#endif //#ifdef TALA_PROTO_SMS_INDIA_PHONE_NUMBER



unsigned char sms1MsgCount = 0;
unsigned char sms2MsgCount = 0;
boolean gAlarm1State = false;
boolean gAlarm2State = false;

//#include "arduino_secrets.h" 
// Please enter your sensitive data in the Secret tab or arduino_secrets.h

// PIN USAG
#define DI_PIN_TEST_ALARM_1  (8u)
#define DI_PIN_TEST_ALARM_2  (9u)

#define DI_PIN_FROM_SLICER   (2u)
#define DI_INTERRUPT_PIN_FROM_SLICER (A2)
#define DI_FROM_RUN_INSTALL  (0u)
#define DI_FROM_DOOR_OPEN    (3u)
#define DO_TO_LDCO_OFF       (4u)
#define DO_TO_MICROCONTR_OFF (1u)
#define DO_TO_BUZZER         (5u)
#define AI_FROM_BTY_MON      (A0)

#define MANCHSTER_DECODED_ONE       (10u)
#define MANCHSTER_DECODED_ZERO      (20u)
unsigned int gManchesterDecodedBitState = 0;

// how many times per second the TC5_Handler() function gets 
// called per second basically
#define SAMPLE_RATE (2000u) // 500 micro second sampling interval 
unsigned int gTc5FiveHundredMicroSecondIsrCount = 0;
#define LED_BLINK_HALF_PERIOD_MS (1000u) // 0.5 second
unsigned int gTc5FiveHundredMicroSecondIsrLedBlinkCount = 0;
unsigned int gTc5PrevFiveHundredMicroSecondIsrLedBlinkCount = 0;
bool ledBlinkState = false; //just for an example 
bool debugState = true;
unsigned int gRxCrcFailCount = 0;

// Manchester Decoded Bit Index
unsigned char gManchesterDecodedBitIndex = 0;
///////////////////////// For 1st bit
#define DEFAULT_1ST_BIT_TRANSITION_CENTRE_COUNT_IN_ISR   (20u)  // 10 mili seconds as 500 microsecond ISR
#define MAXIMUM_1ST_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR (10u)  // 5 mili seconds as 500 microsecond ISR
#define START_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR     (DEFAULT_1ST_BIT_TRANSITION_CENTRE_COUNT_IN_ISR-MAXIMUM_1ST_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR+1) 
#define END_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR       (DEFAULT_1ST_BIT_TRANSITION_CENTRE_COUNT_IN_ISR+MAXIMUM_1ST_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR-1) 
////////////////////////// For 2nd, 3rd, 4th, 6th, 7th and 8th bit
#define DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR       (16u) // 8 mili seconds as 500 microsecond ISR
#define MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR     (7u)  // 3.5 mili seconds as 500 microsecond ISR
#define START_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR         (DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR-MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR+1) 
#define END_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR           (DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR+MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR-1) 
///////////////////////// For 5th bit
#define DEFAULT_5TH_BIT_TRANSITION_CENTRE_COUNT_IN_ISR   (17u) // 8.5 mili seconds as 500 microsecond ISR
#define MAXIMUM_5TH_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR (7u)  // 3.5 mili seconds as 500 microsecond ISR
#define START_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR     (DEFAULT_5TH_BIT_TRANSITION_CENTRE_COUNT_IN_ISR-MAXIMUM_5TH_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR+1) 
#define END_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR       (DEFAULT_5TH_BIT_TRANSITION_CENTRE_COUNT_IN_ISR+MAXIMUM_5TH_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR-1) 

// Alloued values of Manchester Decoded Bit Indexs
#define MANCHESTER_DECODED_BIT0_INDEX (1u)
#define MANCHESTER_DECODED_BIT1_INDEX (2u)
#define MANCHESTER_DECODED_BIT2_INDEX (3u)
#define MANCHESTER_DECODED_BIT3_INDEX (4u)
#define MANCHESTER_DECODED_BIT4_INDEX (5u)
#define MANCHESTER_DECODED_BIT5_INDEX (6u)
#define MANCHESTER_DECODED_BIT6_INDEX (7u)
#define MANCHESTER_DECODED_BIT7_INDEX (8u)
// Chabi manchester samples to receive - D.Lahiri - consuming large RAM - but no option D.Lahiri 11Nov2018
#define NUMBER_OF_500us_SAMPLES_PER_BYTE (256u) // Byte type = 7 ms (bit time) x 8 = 56 ms => 56 x2 = 112 samples
#define NUMBER_OF_500us_SAMPLES_QUEUES   (3u)   // 3 queues per byte
unsigned char gManchesterSamplesByte1[NUMBER_OF_500us_SAMPLES_QUEUES][NUMBER_OF_500us_SAMPLES_PER_BYTE];
unsigned char gManchesterSamplesByte2[NUMBER_OF_500us_SAMPLES_QUEUES][NUMBER_OF_500us_SAMPLES_PER_BYTE];
unsigned char gManchesterSamplesByte3[NUMBER_OF_500us_SAMPLES_QUEUES][NUMBER_OF_500us_SAMPLES_PER_BYTE];
unsigned char gManchesterSamplesByteCrc[NUMBER_OF_500us_SAMPLES_QUEUES][NUMBER_OF_500us_SAMPLES_PER_BYTE];
unsigned int gManchesterSampleIndex = 0;
unsigned int gManchesterSampleQueueIndex = 0;
unsigned int gManchesterSampleEstimateQueueIndex = 0;

// Chabi code to receive
#define NUM_OF_CHABI_CODE_BYTES NUM_OF_TALA_CODE_BYTES
typedef struct {
  unsigned char bit_0: 1;
  unsigned char bit_1: 1;
  unsigned char bit_2: 1;
  unsigned char bit_3: 1;
  unsigned char bit_4: 1;
  unsigned char bit_5: 1;
  unsigned char bit_6: 1;
  unsigned char bit_7: 1;
} _ChabiRxByteType;
typedef union {
   unsigned char ChabiRxByte;
  _ChabiRxByteType ChabiRxByteInBits;
} ChabiRxDataType;
ChabiRxDataType chabiRxCode[NUM_OF_CHABI_CODE_BYTES];
ChabiRxDataType gReceivedCrcValueFromChabi;

unsigned char gCurrent000ManchesterBit = 0;
unsigned char gPreMinus01ManchesterBit = 0;
unsigned char gPreMinus02ManchesterBit = 0;
unsigned char gPreMinus03ManchesterBit = 0;
unsigned char gPreMinus04ManchesterBit = 0;
unsigned char gPreMinus05ManchesterBit = 0;
unsigned char gPreMinus06ManchesterBit = 0;
unsigned char gEstimatedManchesterBit1stHalf = 0;
unsigned char gEstimatedManchesterBit2ndHalf = 0;
unsigned char gEstimatedManchesterBit = 0;
unsigned int gChabiSampleOnesCount = 0;
unsigned int gChabiSampleZerosCount = 0;
unsigned int gChabiThresholdCount = 0;

#define CHABI_SAMPLES_THRESHOLD_COUNT           (4u)   // Number of 1's in preamble
#define CHABI_SAMPLES_ZEROS_COUNT               (130u) // 65 ms < 100 ms
#define CHABI_SAMPLES_ONES_COUNT                (50u)  // 25 ms < 30 ms
#define CHABI_SAMPLES_ERROR_COUNT               (240u) // 120 ms > 100 ms
#define CHABI_SAMPLES_BYTE_GAP_ZEROS_COUNT      (24u)  // 12 ms < 15 ms

boolean gIsChabiCodeEstimated = false;
boolean gIsChabiLastByteReceived = false;
boolean gIsChabiSamplesToBeEstimated = false;
// Receiver State 
enum ChabiReceiverStateType{NOT_KNOWN, CHABI_OFF, CHABI_PREAMBLE_AND_THRESHOLD, CHABI_PRE_RECEIVING, CHABI_RECEIVING_B1, CHABI_RECEIVING_B2, CHABI_RECEIVING_B3, CHABI_RECEIVING_CRC, CHABI_RECEIVED};
enum ChabiReceiverStateType gChabiRxState = NOT_KNOWN;

/*
 * The width of the CRC calculation and result.
 * Modify the typedef for a 16 or 32-bit CRC standard.
 */
typedef unsigned char crc;
#define POLYNOMIAL 0xA7  /* 10100111 */
#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))
/* CRC Lookup table */
crc crcTable[256];
crc gReceivedCrcFromChabi = 0x00;

// RUN INSTALL
#define SYSTEM_NOT_KNOWN  0xFF
#define SYSTEM_RUN        0x01
#define SYSTEM_INSTALL    0x00
unsigned char gRunInstallCurrentState = SYSTEM_NOT_KNOWN;
unsigned char gRunInstallPreviousState = SYSTEM_NOT_KNOWN;

// DOOR OPEN
#define DOOR_NOT_KNOWN  0xFF
#define DOOR_OPEN       0x01
#define DOOR_CLOSE      0x00
unsigned char gDoorCurrentState = DOOR_NOT_KNOWN;
unsigned char gDoorPreviousState = DOOR_NOT_KNOWN;

// initialize the library instances
GSM gGsmAccess;
GSM_SMS gSms;
// Array to hold the number a SMS is retreived from
char senderNumber[20];



boolean gIsTalaChabiCodeMached = true;
unsigned int gTalaChabiCodeMachedCount = 0;
unsigned int gTalaChabiCodeNotMachedCount = 0;
boolean gIsChabiCodeCrcMatchedFirstTime = false;
boolean gIsMicroPowercutTriedFirstTime = false;

// Initialize 
void setup() 
{
  // MKR1000 Rev.1  (DIGITAL PINS USABLE FOR INTERRUPTS) => 0, 1, 4, 5, 6, 7, 8, 9, A1, A2
  
  // Initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);  
  digitalWrite(LED_BUILTIN, LOW);// turn the LED off by making the voltage LOW

  #if 0 // Testing only 
  // Configure input pins
  pinMode(DI_PIN_TEST_ALARM_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DI_PIN_TEST_ALARM_1), diPinTestAlarm2_isr, FALLING);
  pinMode(DI_PIN_TEST_ALARM_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DI_PIN_TEST_ALARM_2), diPinTestAlarm3_isr, FALLING);
  #endif // #if 0 // Testing only 

  // Slicer input from Chabi 
  pinMode(DI_PIN_FROM_SLICER, INPUT);
  pinMode(DI_INTERRUPT_PIN_FROM_SLICER, INPUT);
  attachInterrupt(digitalPinToInterrupt(DI_INTERRUPT_PIN_FROM_SLICER), slicer_isr, RISING);
  
  #ifdef TALA_VIA_SLICER
  pinMode(DI_FROM_RUN_INSTALL, INPUT);
  pinMode(DI_FROM_DOOR_OPEN, INPUT);
  #endif // #ifdef TALA_VIA_SLICER
  #ifdef TALA_DIRECT
  pinMode(DI_FROM_RUN_INSTALL, INPUT_PULLUP);
  pinMode(DI_FROM_DOOR_OPEN, INPUT_PULLUP);
  #endif //#ifdef TALA_DIRECT
  // System RUN INSTALL
  runInstallStateManager();
  // DOOR state
  doorStateManager();
  pinMode(DO_TO_LDCO_OFF, OUTPUT);
  digitalWrite(DO_TO_LDCO_OFF, HIGH);  
  pinMode(DO_TO_MICROCONTR_OFF, OUTPUT); 
  digitalWrite(DO_TO_MICROCONTR_OFF, LOW); 
  pinMode(DO_TO_BUZZER, OUTPUT); 
  //batteryValue = analogRead(AI_FROM_BTY_MON);

  // initialize serial communications and wait for port to open:
  //Serial.end();
  Serial.begin(9600);
  //while (!Serial) 
  //{
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  Serial.println("Set up over ...");  
  tcConfigure(SAMPLE_RATE); //configure the timer to run at <sampleRate>Hertz
  tcStartCounter(); //starts the timer
  
  // Compute CRC table
  crcLutCreate();
  gReceivedCrcValueFromChabi.ChabiRxByte = gReceivedCrcFromChabi;
  // buzzerBeepShortOnetime();
  // Clear sample data
  clearRxManchesterSamplesAllQs();

  // Supported periods on both platforms
  // WDT_PERIOD_1DIV64 = 1/64s
  // WDT_PERIOD_1DIV32 = 1/32s
  // WDT_PERIOD_1DIV16 = 1/16s
  // WDT_PERIOD_1DIV8  = 1/8s
  // WDT_PERIOD_1DIV4  = 1/4s
  // WDT_PERIOD_1DIV2  = 1/2s
  // WDT_PERIOD_1X     = 1s
  // WDT_PERIOD_2X     = 2s
  // WDT_PERIOD_4X     = 4s
  // WDT_PERIOD_8X     = 8s
  // Default parameter is set to WDT_PERIOD_1X = 1s
  // Enable WDT
  //sodaq_wdt_enable(WDT_PERIOD_8X); // Use later - when product phase D.Lahiri 18Jan2019
}

// Buzzer Beep short 1 time
void buzzerBeepShortOnetime(void)
{
  digitalWrite(DO_TO_BUZZER, HIGH);
  delay(100);
  digitalWrite(DO_TO_BUZZER, LOW);
  delay(200);
  sodaq_wdt_reset();
} 
// Buzzer beep welcome 
void buzzerBeepWelcome(void)
{
  buzzerBeepShortOnetime();
  buzzerBeepShortOnetime();
  buzzerBeepShortOnetime();
}
// Buzzer beep door close 
void buzzerBeepDoorClose(void)
{
  buzzerBeepDoorCloseShortOrRxCrcFail();
  delay(150);
  buzzerBeepDoorCloseShortOrRxCrcFail();
  delay(150);
  buzzerBeepDoorCloseShortOrRxCrcFail();
  delay(150);
}
// Buzzer Beep long 1 time
void buzzerBeepLongOnetime(void)
{
  digitalWrite(DO_TO_BUZZER, HIGH);
  delay(200);
  digitalWrite(DO_TO_BUZZER, LOW);
  delay(300);
  sodaq_wdt_reset();
} 

// Buzzer authentication fail 
void buzzerBeepAuthentationFailDoorOpen(void)
{
  buzzerBeepLongOnetime();
  buzzerBeepLongOnetime();
  buzzerBeepLongOnetime();
  buzzerBeepLongOnetime();
  buzzerBeepLongOnetime();
  buzzerBeepLongOnetime();
  buzzerBeepLongOnetime();
  buzzerBeepLongOnetime();
  buzzerBeepLongOnetime();
  buzzerBeepLongOnetime();
}
void buzzerBeepAuthentationFail(void)
{
  buzzerBeepLongOnetime();
}
// Buzzer beep Rx CRC Fail 
void buzzerBeepDoorCloseShortOrRxCrcFail(void)
{
  digitalWrite(DO_TO_BUZZER, HIGH);
  delay(30);
  digitalWrite(DO_TO_BUZZER, LOW);
  sodaq_wdt_reset();
}

// Buzzer beep Rx CRC match 
void buzzerBeepRxCrcMatch(void)
{
  digitalWrite(DO_TO_BUZZER, HIGH);
  delay(30);
  digitalWrite(DO_TO_BUZZER, LOW);
  delay(100);
  digitalWrite(DO_TO_BUZZER, HIGH);
  delay(30);
  digitalWrite(DO_TO_BUZZER, LOW);
  sodaq_wdt_reset();
}

// System RUN INSTALL
void runInstallStateManager(void)
{
  // enum RunInstallStateType{SYSTEM_NOT_KNOWN, SYSTEM_RUN, SYSTEM_INSTALL};
  gRunInstallCurrentState = digitalRead(DI_FROM_RUN_INSTALL);
  if(gRunInstallCurrentState != gRunInstallPreviousState)
  {
    gRunInstallPreviousState = gRunInstallCurrentState;
    printTimeStamp();
    switch(gRunInstallCurrentState)
    {
      case SYSTEM_RUN:
        Serial.println("SYSTEM_RUN");
        break;
      case SYSTEM_INSTALL:
        Serial.println("SYSTEM_INSTALL");
        break;
      default:
        Serial.println("SYSTEM_NOT_KNOWN");
        break;
    }
  }
}

// DOOR OPEN CLOSE
void doorStateManager(void)
{
  // enum DoorStateType{DOOR_NOT_KNOWN, DOOR_OPEN, DOOR_CLOSE}; 
  gDoorCurrentState = digitalRead(DI_FROM_DOOR_OPEN);
  if(gDoorCurrentState != gDoorPreviousState)
  {
    gDoorPreviousState = gDoorCurrentState;
    printTimeStamp();
    switch(gDoorCurrentState)
    {
      case DOOR_OPEN:
        Serial.println("DOOR_OPEN");
        break;
      case DOOR_CLOSE:
        Serial.println("DOOR_CLOSE");
        break;
      default:
        Serial.println("DOOR_NOT_KNOWN");
        break;
    }
  }  
}

// Wait for One minute using delay routine but watchdog enable
void waitForOneMinute()
{
  unsigned int secondCount;
  // 60 times 1 s delay with watchdog reset
  for(secondCount = 1; secondCount <= 60; secondCount++)
  {
    sodaq_wdt_reset();
    delay(1000); // One second delay
  }
}

// GSM Connection check
void connectToGsm()
{
  int c; // For SMS receive
  // connection state
  bool connectedStatus = false;
  // Start GSM connection
  while (!connectedStatus) 
  {
    if (gGsmAccess.begin(PINNUMBER) == GSM_READY) 
    {
      connectedStatus = true;
    } 
    else 
    {
      Serial.println("Not connected");
      delay(1000);
      sodaq_wdt_reset();
    }
  }
  Serial.println("GSM initialized");
  //Serial.println("Waiting for any messages");
  // Get remote number
  //gSms.remoteNumber(senderNumber, 20);
  //Serial.print ("Message received from: ");
  //Serial.println(senderNumber);
  // An example of message disposal
  // Any messages starting with # should be discarded
  if (gSms.peek() == '#') 
  {
    Serial.println("Discarded SMS");
    gSms.flush();
  }
  // Read message bytes and print them
  while ((c = gSms.read()) != -1) 
  {
    Serial.print((char)c);
  }
  //Serial.println("\nEND OF MESSAGE");
  // Delete message from modem memory
  gSms.flush();
  //Serial.println("MESSAGE DELETED");  
}

// Loop for ever
void loop() 
{

  // tcDisable(); //This function can be used anywhere if you need to stop/pause the timer
  // tcReset(); //This function should be called everytime you stop the timer

  // Reset Watchdog
  sodaq_wdt_reset();
  
  // System RUN INSTALL
  runInstallStateManager();

  // DOOR state
  doorStateManager();

  // Estimate Chabi code from Samples at Door close 
  if(gDoorCurrentState == DOOR_CLOSE)
  { 
    if((gIsChabiSamplesToBeEstimated == true) && (gChabiRxState == CHABI_RECEIVED))
    {
      gIsChabiSamplesToBeEstimated = false;
      estimateRxBitsFromManchesterSamples();
      gIsChabiCodeEstimated = true;
    }
  }
  else // gDoorCurrentState == DOOR_OPEN
  {
    Serial.print("gChabiRxState = ");
    Serial.println(gChabiRxState, HEX);  
    // Intrusion - no "chabi" detected but DOOR is OPEN
    // NOT_KNOWN, CHABI_OFF, CHABI_PREAMBLE_AND_THRESHOLD, CHABI_PRE_RECEIVING, 
    // CHABI_RECEIVING_B1, CHABI_RECEIVING_B2, CHABI_RECEIVING_B3, CHABI_RECEIVING_CRC, CHABI_RECEIVED};
    if((gChabiRxState == NOT_KNOWN) || (gChabiRxState == CHABI_OFF) || (gChabiRxState == CHABI_PREAMBLE_AND_THRESHOLD) )
    {
      // Wait for Sometime - to be sure that door is still open
      delay(1000);
      doorStateManager();
      // Still now door is open and No Chabi
      if(((gChabiRxState == NOT_KNOWN) || (gChabiRxState == CHABI_OFF) || (gChabiRxState == CHABI_PREAMBLE_AND_THRESHOLD)) && (gDoorCurrentState == DOOR_OPEN))
      {
        buzzerBeepAuthentationFailDoorOpen(); // This is also done infinnitely after SMS text
        gIsTalaChabiCodeMached = false; // Intrusion - no "chabi" detected but DOOR is OPEN
      }
    }
  }

  // Print if Chabi code along with CRC as received  
  // calculated CRC value of tala code  
  //crc calculatedTalaCrcValue;
  //calculatedTalaCrcValue = crcFromLut(talaCode, NUM_OF_TALA_CODE_BYTES);
  // Received and Calculated CRC value of chabi 
  crc calculatedCrcValueFromChabi;
  //gReceivedCrcFromChabi = gReceivedCrcValueFromChabi.ChabiRxByte;
  if(gIsChabiCodeEstimated == true)
  {
    // calculate CRC value of chabi code received   
    calculatedCrcValueFromChabi = crcFromLut(&chabiRxCode[0].ChabiRxByte, NUM_OF_CHABI_CODE_BYTES);
    if((gReceivedCrcValueFromChabi.ChabiRxByte == calculatedCrcValueFromChabi) && (gReceivedCrcValueFromChabi.ChabiRxByte != 0))
    {
      buzzerBeepRxCrcMatch();
      printTimeStamp();
      gIsChabiCodeCrcMatchedFirstTime = true;
      Serial.print("Chabi code received :- ");
      for(unsigned char chabiRxByteCount = 0; chabiRxByteCount < NUM_OF_CHABI_CODE_BYTES; chabiRxByteCount++)
      {
        Serial.print(chabiRxCode[chabiRxByteCount].ChabiRxByte, HEX);
        Serial.print(" ");
      }
      Serial.print(gReceivedCrcValueFromChabi.ChabiRxByte, HEX);
      Serial.print(" ");
      Serial.print("- CRC Calculated :- ");
      Serial.print(calculatedCrcValueFromChabi, HEX);
      Serial.print(" Rx Okey ");
      Serial.print(" - Binary =>  ");
      for(unsigned char chabiRxByteCount = 0; chabiRxByteCount < NUM_OF_CHABI_CODE_BYTES; chabiRxByteCount++)
      {
        Serial.print(chabiRxCode[chabiRxByteCount].ChabiRxByte, BIN);
        Serial.print(" ");
      }
      Serial.print(gReceivedCrcValueFromChabi.ChabiRxByte, BIN);
      Serial.print("\n");

      // Check Authentication
      if((talaCode[0] == chabiRxCode[0].ChabiRxByte) &&
         (talaCode[1] == chabiRxCode[1].ChabiRxByte) &&
         (talaCode[2] == chabiRxCode[2].ChabiRxByte)) // tala chabi code matched 
      {
        gTalaChabiCodeMachedCount++;
        delay(500);
        buzzerBeepWelcome();
        sodaq_wdt_reset();
        delay(5000); // Wait for door open lst repeat 
        sodaq_wdt_reset();
        doorStateManager();
        while(gDoorCurrentState == DOOR_OPEN) // Door open
        {
          buzzerBeepDoorClose();
          delay(3000); // Wait for door close else repeat 
          sodaq_wdt_reset();
          doorStateManager();
        }
        gIsTalaChabiCodeMached = true;
      }
      else // tala chabi code not matched 
      {
        gTalaChabiCodeNotMachedCount++;
        delay(2000); // Wait for door open
        doorStateManager();
        if(gDoorCurrentState == DOOR_OPEN) 
        {
          buzzerBeepAuthentationFailDoorOpen(); // This is also done infinnitely after SMS text
          gIsTalaChabiCodeMached = false;
        }
      }
    }
    else
    {
      gRxCrcFailCount++;
      if(gIsMicroPowercutTriedFirstTime == true)
      {
         gIsChabiCodeCrcMatchedFirstTime = true;
      }
      Serial.print("Rx Error xxxxx - Count :- 0x");
      Serial.print(gRxCrcFailCount, HEX);
      Serial.print(" - received => ");
      for(unsigned char chabiRxByteCount = 0; chabiRxByteCount < NUM_OF_CHABI_CODE_BYTES; chabiRxByteCount++)
      {
        Serial.print(chabiRxCode[chabiRxByteCount].ChabiRxByte, HEX);
        Serial.print(" ");
      }
      Serial.print(gReceivedCrcValueFromChabi.ChabiRxByte, HEX);
      Serial.print(" ");
      Serial.print(" =>  ");
      for(unsigned char chabiRxByteCount = 0; chabiRxByteCount < NUM_OF_CHABI_CODE_BYTES; chabiRxByteCount++)
      {
        Serial.print(chabiRxCode[chabiRxByteCount].ChabiRxByte, BIN);
        Serial.print(" ");
      }
      Serial.print(gReceivedCrcValueFromChabi.ChabiRxByte, BIN);
      Serial.print(" => gChabiRxState = ");
      Serial.println(gChabiRxState, HEX);  
      //Serial.print("\n");
      buzzerBeepDoorCloseShortOrRxCrcFail();
    }
    // For next receive in install and run mode if Chabi is not removed
    gIsChabiCodeEstimated = false;   
  }

  // Check if Chabi Tala not matched
  if(gIsTalaChabiCodeMached == false)
  {
    // Stop the timer
    tcDisable(); //This function can be used anywhere if you need to stop/pause the timer
    tcReset(); //This function should be called everytime you stop the timer
    Serial.println("Chabi code mismatched .. ");
    gIsTalaChabiCodeMached = true; // To avoid re init
    // initialize serial communications and wait for port to open:
    //Serial.begin(9600);
    //while (!Serial) 
    //{
      //; // wait for serial port to connect. Needed for native USB port only
    //}

    
    // Enable which phone number you want to send SMS 
        
    // Send SMS to 1st phone number
    #ifdef TALA_PROTO_SMS_USA_PHONE_NUMBER
      connectToGsm();
      Serial.print("Going for GSM SMS to ");
      Serial.print(gPhoneNumber1);
      Serial.println(" (Dr Somnath Mukherjee) ");
      Serial.println("\nCHABI MISMATCH !");
      gSms.beginSMS(PHONE_NUM_1);
      gSms.print(gTextForChabiAlarm1);
      gSms.endSMS();
      Serial.print("SMS SEND COMPLETE for Dr Somnath Mmukherjee => ");
      Serial.println(gTextForChabiAlarm1);
      sodaq_wdt_disable();
      delay(3000); // wait for three seconds to avoid multiple messages
    #endif // #ifdef TALA_PROTO_SMS_USA_PHONE_NUMBER
    // Send SMS to 2nd phone number
    #ifdef TALA_PROTO_SMS_INDIA_PHONE_NUMBER
      connectToGsm();
      Serial.print("Going for GSM SMS to ");
      Serial.print(gPhoneNumber2);
      Serial.println(" (Mr Debojyoti Lahiri) ");
      Serial.println("\nCHABI MISMATCH !");
      gSms.beginSMS(PHONE_NUM_2);
      gSms.print(gTextForChabiAlarm2);
      gSms.endSMS();
      Serial.print("SMS SEND COMPLETE for Mr Debojyoti Lahiri => ");
      Serial.println(gTextForChabiAlarm2);
      sodaq_wdt_disable();
      delay(3000); // wait for three seconds to avoid multiple messages
    #endif //#ifdef TALA_PROTO_SMS_INDIA_PHONE_NUMBER

    #ifdef TALA_PROTO_SMS_USA_TEST_PHONE_NUMBER // Now Victor - chnage
      connectToGsm();
      Serial.print("Going for GSM SMS to ");
      Serial.print(gPhoneNumber3);
      Serial.println(" (Mr Victor) ");
      Serial.println("\nCHABI MISMATCH !");
      gSms.beginSMS(PHONE_NUM_3);
      gSms.print(gTextForChabiAlarm3);
      gSms.endSMS();
      Serial.print("SMS SEND COMPLETE for Mr Victor => ");
      Serial.println(gTextForChabiAlarm3);
      sodaq_wdt_disable();
      delay(3000); // wait for three seconds to avoid multiple messages
    #endif // #ifdef TALA_PROTO_SMS_USA_TEST_PHONE_NUMBER
    
    // Set buzzer infinitely till manual reset or battery goes off
    while(1)
    {
      buzzerBeepAuthentationFailDoorOpen();
    }
    
    // The code of WDT reset is not used currently
    // Enable WDT
    sodaq_wdt_enable(WDT_PERIOD_8X);
    waitForOneMinute(); // wait for one minute to complete message and avoid multipe message
    for(;;); // Watchdog reset
  }
  
  // Check battery low
  // TBD
  // Send SMS text if battery low
  // TBD

  // Microcontroller off to save battery power 
  runInstallStateManager();
  if((gRunInstallCurrentState == SYSTEM_RUN) && (gIsChabiCodeCrcMatchedFirstTime == true))
  {
    
    printTimeStamp();
    Serial.println("System in RUM mode => Going to cut off Micro Power to save battery ......");// For Debugging only => D.Lahiri 11Feb2019
    delay(1000); // wait for 1 second 
    gIsChabiCodeCrcMatchedFirstTime = false;
    gIsMicroPowercutTriedFirstTime = true; 
    digitalWrite(DO_TO_LDCO_OFF, LOW); // LDCO on
    digitalWrite(DO_TO_MICROCONTR_OFF, HIGH);// Micro off  
    delay(1000); // wait for 1 second 
  }
}

// Print Time stamp function
void printTimeStamp(void)
{
  Serial.print("Time: ");
  Serial.print(millis()/(1000.0));
  Serial.print(" seconds - ");
}

/* Compute the CRC Table */
void crcLutCreate(void)
{
  crc remainder;
  /* Compute the remainder of each possible dividend. */
  for (int dividend = 0; dividend < 256; ++dividend)
  {
    /* Start with the dividend followed by zeros.*/
    remainder = dividend << (WIDTH - 8);
    /* Perform modulo-2 division, a bit at a time. */
    for (unsigned char bit = 8; bit > 0; --bit)
    {
      /* Try to divide the current data bit.*/
      if (remainder & TOPBIT)
      {
        remainder = (remainder << 1) ^ POLYNOMIAL;
      }
      else
      {
        remainder = (remainder << 1);
      }
    }
    /* Store the result into the table.*/
    crcTable[dividend] = remainder;
  }
}/* crcLutCreate() */

/* Compute the CRC from Lookup table */
crc crcFromLut(unsigned char const message[], int nBytes)
{
  unsigned char data;
  crc remainder = 0;
  /* Divide the message by the polynomial, a byte at a time.*/
  for (int byte = 0; byte < nBytes; ++byte)
  {
    data = message[byte] ^ (remainder >> (WIDTH - 8));
    remainder = crcTable[data] ^ (remainder << 8);
  }
  /* The final remainder is the CRC.*/
  return (remainder);
} /* crcFromLut() */

#if 0 // Testing only 
// Digital input pin test interrupt service routine 2
void diPinTestAlarm2_isr(void)
{
  gAlarm1State = true;
}

// Digital input pin test interrupt service routine 3
void diPinTestAlarm3_isr(void)
{
  gAlarm2State = true;
}
#endif // #if 0 // Testing only 

// Clear Received Manchester Samples for all Queues
void clearRxManchesterSamplesAllQs(void)
{
  unsigned int localManchesterSampleIndex, localManchesterSampleQueueIndex;
  for(localManchesterSampleQueueIndex = 0; localManchesterSampleQueueIndex < NUMBER_OF_500us_SAMPLES_QUEUES; localManchesterSampleQueueIndex++)
  {
    for(localManchesterSampleIndex = 0; localManchesterSampleIndex < NUMBER_OF_500us_SAMPLES_PER_BYTE; localManchesterSampleIndex++)
    {
      gManchesterSamplesByte1[localManchesterSampleQueueIndex][localManchesterSampleIndex] = 0;
      gManchesterSamplesByte2[localManchesterSampleQueueIndex][localManchesterSampleIndex] = 0;
      gManchesterSamplesByte3[localManchesterSampleQueueIndex][localManchesterSampleIndex] = 0;
      gManchesterSamplesByteCrc[localManchesterSampleQueueIndex][localManchesterSampleIndex] = 0;
    }
  }
}

// Clear Received Manchester Samples for this Queue
void clearRxManchesterSamplesThisQ(void)
{
  unsigned int localManchesterSampleIndex;
  for(localManchesterSampleIndex = 0; localManchesterSampleIndex < NUMBER_OF_500us_SAMPLES_PER_BYTE; localManchesterSampleIndex++)
  {
    gManchesterSamplesByte1[gManchesterSampleQueueIndex][localManchesterSampleIndex] = 0;
    gManchesterSamplesByte2[gManchesterSampleQueueIndex][localManchesterSampleIndex] = 0;
    gManchesterSamplesByte3[gManchesterSampleQueueIndex][localManchesterSampleIndex] = 0;
    gManchesterSamplesByteCrc[gManchesterSampleQueueIndex][localManchesterSampleIndex] = 0;
  }
}
// Slicer input Change ISR 
void slicer_isr(void)
{
  switch(gChabiRxState)
  {
    case CHABI_PRE_RECEIVING:
      gChabiRxState = CHABI_RECEIVING_B1;
      gTc5FiveHundredMicroSecondIsrCount = 0;
      gIsChabiLastByteReceived = false;
      gManchesterSampleIndex = 0;
      break;
    case CHABI_RECEIVING_B1:
      if(gIsChabiLastByteReceived == true)
      {
        gChabiRxState = CHABI_RECEIVING_B2;
        gTc5FiveHundredMicroSecondIsrCount = 0;
        gIsChabiLastByteReceived = false;
        gManchesterSampleIndex = 0;
      }
      break;
    case CHABI_RECEIVING_B2:
      if(gIsChabiLastByteReceived == true)
      {
        gChabiRxState = CHABI_RECEIVING_B3;
        gTc5FiveHundredMicroSecondIsrCount = 0;
        gIsChabiLastByteReceived = false;
        gManchesterSampleIndex = 0;
      }
      break;
    case CHABI_RECEIVING_B3:
      if(gIsChabiLastByteReceived == true)
      {
        gChabiRxState = CHABI_RECEIVING_CRC;
        gTc5FiveHundredMicroSecondIsrCount = 0;
        gIsChabiLastByteReceived = false;
        gManchesterSampleIndex = 0;
        //Serial.println(" -Crc- ");
      }
      break;
    default:
      break;
  }
}

// this function gets called by the interrupt at <sampleRate> Hertz
// Currently set to 500 micro second
void TC5_Handler (void) 
{
  noInterrupts();
  //enum ChabiReceiverStateType{NOT_KNOWN, CHABI_OFF, CHABI_PREAMBLE_AND_THRESHOLD, CHABI_PRE_RECEIVING, 
  //                            CHABI_RECEIVING_B1, CHABI_RECEIVING_B2, CHABI_RECEIVING_B3, CHABI_RECEIVING_CRC, CHABI_RECEIVED};

  // Read slicer output sample 
  gCurrent000ManchesterBit = digitalRead(DI_PIN_FROM_SLICER);
  
  // Adjust last consicutive 1's and 0's counts
  if(gCurrent000ManchesterBit == 0x01) 
  {
    gChabiSampleOnesCount++;
    gChabiSampleZerosCount = 0;
  }
  else
  {
    gChabiSampleOnesCount = 0;
    gChabiSampleZerosCount++;
  }

  // Adjust state to unknown when no chabi is there or some slicer problem in tala
  if((gChabiSampleZerosCount >= CHABI_SAMPLES_ERROR_COUNT)||(gChabiSampleOnesCount >= CHABI_SAMPLES_ERROR_COUNT))
  {
    gChabiRxState = NOT_KNOWN;
    gTc5FiveHundredMicroSecondIsrCount = 0;
    gChabiSampleZerosCount  = 0;
    gChabiSampleOnesCount = 0;
    gChabiThresholdCount = 0;
  }
  
  // Adjust Receiver state till CHABI_RECEIVING
  switch(gChabiRxState)
  {
    case NOT_KNOWN:
    case CHABI_RECEIVED:
      if(gChabiSampleZerosCount >= CHABI_SAMPLES_ZEROS_COUNT)
      {
        gChabiRxState = CHABI_OFF;
        gChabiSampleZerosCount = 0;
      }
      break;
    case CHABI_OFF:
      clearRxManchesterSamplesThisQ();
      if(gChabiSampleOnesCount >= CHABI_SAMPLES_ONES_COUNT)
      {
        gChabiRxState = CHABI_PREAMBLE_AND_THRESHOLD;
        gChabiSampleOnesCount = 0;
      }
      gChabiThresholdCount = 0;
      break;
    case CHABI_PREAMBLE_AND_THRESHOLD:
      if((gCurrent000ManchesterBit == 1)&&(gPreMinus01ManchesterBit == 0))
      {
        // Calculate the number of rising edges i.e. 0 to 1
        gChabiThresholdCount++;
        // The chabi code starts after 4 bits i.e on 5th rising edge
        // We are detecting 16th rising edge here .. will change state to CHABI_RECEIVING in slicer isr
        if(gChabiThresholdCount == CHABI_SAMPLES_THRESHOLD_COUNT)
        { 
          gChabiRxState = CHABI_PRE_RECEIVING;
          //digitalWrite(LED_BUILTIN, HIGH);// turn the LED on by making the voltage HIGH
        }
      }
      gTc5FiveHundredMicroSecondIsrCount = 0;
      gChabiSampleZerosCount = 0;
      gChabiSampleOnesCount = 0;
      break;
    default:
      break;
  }

  // Adjust Previous received samples and estimate the bit 
  gPreMinus06ManchesterBit = gPreMinus05ManchesterBit;
  gPreMinus05ManchesterBit = gPreMinus04ManchesterBit;
  gPreMinus04ManchesterBit = gPreMinus03ManchesterBit;
  gPreMinus03ManchesterBit = gPreMinus02ManchesterBit;
  gPreMinus02ManchesterBit = gPreMinus01ManchesterBit;
  gPreMinus01ManchesterBit = gCurrent000ManchesterBit;

  // Receive Chabi Manchester bits in CHABI_RECEIVING state
  if(gChabiRxState == CHABI_RECEIVING_B1)
  {
    gManchesterSamplesByte1[gManchesterSampleQueueIndex][gManchesterSampleIndex] = gCurrent000ManchesterBit;
    gManchesterSampleIndex++;
    if(gChabiSampleZerosCount >= CHABI_SAMPLES_BYTE_GAP_ZEROS_COUNT)
    {
      gIsChabiLastByteReceived = true;
      gTc5FiveHundredMicroSecondIsrCount = 0;
      gChabiSampleZerosCount = 0;
      gChabiSampleOnesCount = 0;
    }
  }
  
  // Receive Chabi Manchester bits in CHABI_RECEIVING state
  if(gChabiRxState == CHABI_RECEIVING_B2)
  {
    gManchesterSamplesByte2[gManchesterSampleQueueIndex][gManchesterSampleIndex] = gCurrent000ManchesterBit;
    gManchesterSampleIndex++;
    if(gChabiSampleZerosCount >= CHABI_SAMPLES_BYTE_GAP_ZEROS_COUNT)
    {
      gIsChabiLastByteReceived = true;
      gTc5FiveHundredMicroSecondIsrCount = 0;
      gChabiSampleZerosCount = 0;
      gChabiSampleOnesCount = 0;
    }
  }

  // Receive Chabi Manchester bits in CHABI_RECEIVING state
  if(gChabiRxState == CHABI_RECEIVING_B3)
  {
    gManchesterSamplesByte3[gManchesterSampleQueueIndex][gManchesterSampleIndex] = gCurrent000ManchesterBit;
    gManchesterSampleIndex++;
    if(gChabiSampleZerosCount > CHABI_SAMPLES_BYTE_GAP_ZEROS_COUNT)
    {
      gIsChabiLastByteReceived = true;
      gTc5FiveHundredMicroSecondIsrCount = 0;
      gChabiSampleZerosCount = 0;
      gChabiSampleOnesCount = 0;
    }
  }

  // Receive Chabi Manchester bits in CHABI_RECEIVING state
  if(gChabiRxState == CHABI_RECEIVING_CRC)
  {
    gManchesterSamplesByteCrc[gManchesterSampleQueueIndex][gManchesterSampleIndex] = gCurrent000ManchesterBit;
    gManchesterSampleIndex++;
    if(gChabiSampleZerosCount > CHABI_SAMPLES_BYTE_GAP_ZEROS_COUNT)
    {
      gIsChabiLastByteReceived = true;
      gTc5FiveHundredMicroSecondIsrCount = 0;
      gChabiSampleZerosCount = 0;
      gChabiSampleOnesCount = 0;
      gIsChabiSamplesToBeEstimated = true;
      gChabiRxState = CHABI_RECEIVED;
      // Adjust queue index
      gManchesterSampleEstimateQueueIndex = gManchesterSampleQueueIndex;
      gManchesterSampleQueueIndex++;
      if(gManchesterSampleQueueIndex >= NUMBER_OF_500us_SAMPLES_QUEUES)
      {
        gManchesterSampleQueueIndex = 0;      
      }
      //digitalWrite(LED_BUILTIN, LOW);// turn the LED off by making the voltage LOW
    }
  }

  // LED blink to check if ISR is running
  gTc5FiveHundredMicroSecondIsrLedBlinkCount++;
  
  if(gTc5FiveHundredMicroSecondIsrLedBlinkCount == LED_BLINK_HALF_PERIOD_MS)
  {
    gTc5FiveHundredMicroSecondIsrLedBlinkCount = 0;
    ledBlinkState = !ledBlinkState;
    if(ledBlinkState == true) 
    {
      digitalWrite(LED_BUILTIN,HIGH);
    } 
    else 
    {
      digitalWrite(LED_BUILTIN,LOW);
    }
  }
  else
  {
    // Do nothing
  }  

  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
  interrupts();
}

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

// Configures the TC to generate output events at the sample frequency.
// Configures the TC in Frequency Generation mode, with an event output once
// each time the sample frequency period expires.
void tcConfigure(int sampleRate)
{
 // Enable GCLK for TCC2 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); // reset TC5

 // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 // set prescaler and enable TC5
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
 // set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
 TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

// Function that is used to check if TC5 is done syncing
// returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

// This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

// Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

// disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}



// Estimate bits from  Received Manchester Samples
void estimateRxBitsFromManchesterSamples(void)
{
  unsigned int receivedSampleIndex = 0, lastTransitionSampleIndex = 0, thisTransitionSampleIndex = 0, minimumTransitionSampleIndex = 0;
  //boolean isByteEstimated = false, isBitEstimated = false;
  unsigned char bitIndex = 0;
  unsigned char currentSampleValue = 0, previousSampleValue = 0;
  unsigned char thisTransitionSampleDistance = 0, minimumTransitionSampleDistance = 0;
  
  // Estimate Byte # 1 ******************************************
  chabiRxCode[0].ChabiRxByte = 0x00; // Init this byte with zeros
  lastTransitionSampleIndex = 0; // To be adjusted after each bit estimate
  // For 1st bit in this byte 
  bitIndex = 0;
  minimumTransitionSampleDistance = MAXIMUM_1ST_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
  for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR); \
      receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR);\
      receivedSampleIndex++)
  {
    // Read sample values
    currentSampleValue = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
    previousSampleValue = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
    // Detect transition 
    if(currentSampleValue !=  previousSampleValue)
    {
      thisTransitionSampleIndex = receivedSampleIndex;
      thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_1ST_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
      // Detect minimum distance from default theoritical transition 
      if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
      {
        minimumTransitionSampleDistance = thisTransitionSampleDistance;
        minimumTransitionSampleIndex = thisTransitionSampleIndex;
      }
    }
  }
  // Adjust for next bit 
  lastTransitionSampleIndex = minimumTransitionSampleIndex;
  // Estimate this bit 
  chabiRxCode[0].ChabiRxByteInBits.bit_7 = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
  // For next 3 bits (2nd, 3rd and 4th)
  for(bitIndex = 1; bitIndex <= 3; bitIndex++)
  {
    minimumTransitionSampleDistance = MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
    for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR); \
        receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR);\
        receivedSampleIndex++)
    {
      // Read sample values
      currentSampleValue = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
      previousSampleValue = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
      // Detect transition 
      if(currentSampleValue !=  previousSampleValue)
      {
        thisTransitionSampleIndex = receivedSampleIndex;
        thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
        // Detect minimum distance from default theoritical transition 
        if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
        {
          minimumTransitionSampleDistance = thisTransitionSampleDistance;
          minimumTransitionSampleIndex = thisTransitionSampleIndex;
        }
      }
    }
    // Adjust for next bit 
    lastTransitionSampleIndex = minimumTransitionSampleIndex;
    // Estimate this bit 
    switch(bitIndex)
    { 
      case 1:
        chabiRxCode[0].ChabiRxByteInBits.bit_6 = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 2:
        chabiRxCode[0].ChabiRxByteInBits.bit_5 = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 3:
        chabiRxCode[0].ChabiRxByteInBits.bit_4 = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      default:
        break;
    }
  }
  // For 5th bit in this byte 
  bitIndex = 4;
  minimumTransitionSampleDistance = MAXIMUM_5TH_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
  for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR); \
      receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR);\
      receivedSampleIndex++)
  {
    // Read sample values
    currentSampleValue = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
    previousSampleValue = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
    // Detect transition 
    if(currentSampleValue !=  previousSampleValue)
    {
      thisTransitionSampleIndex = receivedSampleIndex;
      thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_5TH_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
      // Detect minimum distance from default theoritical transition 
      if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
      {
        minimumTransitionSampleDistance = thisTransitionSampleDistance;
        minimumTransitionSampleIndex = thisTransitionSampleIndex;
      }
    }
  }
  // Adjust for next bit 
  lastTransitionSampleIndex = minimumTransitionSampleIndex;
  // Estimate this bit 
  chabiRxCode[0].ChabiRxByteInBits.bit_3 = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
  // For last 3 bits (6th, 7th and 8th) 
  for(bitIndex = 5; bitIndex <= 7; bitIndex++)
  {
    minimumTransitionSampleDistance = MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
    for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR); \
        receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR);\
        receivedSampleIndex++)
    {
      // Read sample values
      currentSampleValue = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
      previousSampleValue = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
      // Detect transition 
      if(currentSampleValue !=  previousSampleValue)
      {
        thisTransitionSampleIndex = receivedSampleIndex;
        thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
        // Detect minimum distance from default theoritical transition 
        if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
        {
          minimumTransitionSampleDistance = thisTransitionSampleDistance;
          minimumTransitionSampleIndex = thisTransitionSampleIndex;
        }
      }
    }
    // Adjust for next bit 
    lastTransitionSampleIndex = minimumTransitionSampleIndex;
    // Estimate this bit 
    switch(bitIndex)
    { 
      case 5:
        chabiRxCode[0].ChabiRxByteInBits.bit_2 = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 6:
        chabiRxCode[0].ChabiRxByteInBits.bit_1 = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 7:
        chabiRxCode[0].ChabiRxByteInBits.bit_0 = gManchesterSamplesByte1[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      default:
        break;
    }
  }
  // Estimate Byte # 1 Done ******************************************
  
  // Estimate Byte # 2 ******************************************
  chabiRxCode[1].ChabiRxByte = 0x00; // Init this byte with zeros
  lastTransitionSampleIndex = 0; // To be adjusted after each bit estimate
  // For 1st bit in this byte 
  bitIndex = 0;
  minimumTransitionSampleDistance = MAXIMUM_1ST_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
  for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR); \
      receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR);\
      receivedSampleIndex++)
  {
    // Read sample values
    currentSampleValue = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
    previousSampleValue = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
    // Detect transition 
    if(currentSampleValue !=  previousSampleValue)
    {
      thisTransitionSampleIndex = receivedSampleIndex;
      thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_1ST_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
      // Detect minimum distance from default theoritical transition 
      if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
      {
        minimumTransitionSampleDistance = thisTransitionSampleDistance;
        minimumTransitionSampleIndex = thisTransitionSampleIndex;
      }
    }
  }
  // Adjust for next bit 
  lastTransitionSampleIndex = minimumTransitionSampleIndex;
  // Estimate this bit 
  chabiRxCode[1].ChabiRxByteInBits.bit_7 = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
  // For next 3 bits (2nd, 3rd and 4th)
  for(bitIndex = 1; bitIndex <= 3; bitIndex++)
  {
    minimumTransitionSampleDistance = MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
    for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR); \
        receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR);\
        receivedSampleIndex++)
    {
      // Read sample values
      currentSampleValue = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
      previousSampleValue = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
      // Detect transition 
      if(currentSampleValue !=  previousSampleValue)
      {
        thisTransitionSampleIndex = receivedSampleIndex;
        thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
        // Detect minimum distance from default theoritical transition 
        if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
        {
          minimumTransitionSampleDistance = thisTransitionSampleDistance;
          minimumTransitionSampleIndex = thisTransitionSampleIndex;
        }
      }
    }
    // Adjust for next bit 
    lastTransitionSampleIndex = minimumTransitionSampleIndex;
    // Estimate this bit 
    switch(bitIndex)
    { 
      case 1:
        chabiRxCode[1].ChabiRxByteInBits.bit_6 = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 2:
        chabiRxCode[1].ChabiRxByteInBits.bit_5 = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 3:
        chabiRxCode[1].ChabiRxByteInBits.bit_4 = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      default:
        break;
    }
  }
  // For 5th bit in this byte 
  bitIndex = 4;
  minimumTransitionSampleDistance = MAXIMUM_5TH_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
  for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR); \
      receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR);\
      receivedSampleIndex++)
  {
    // Read sample values
    currentSampleValue = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
    previousSampleValue = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
    // Detect transition 
    if(currentSampleValue !=  previousSampleValue)
    {
      thisTransitionSampleIndex = receivedSampleIndex;
      thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_5TH_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
      // Detect minimum distance from default theoritical transition 
      if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
      {
        minimumTransitionSampleDistance = thisTransitionSampleDistance;
        minimumTransitionSampleIndex = thisTransitionSampleIndex;
      }
    }
  }
  // Adjust for next bit 
  lastTransitionSampleIndex = minimumTransitionSampleIndex;
  // Estimate this bit 
  chabiRxCode[1].ChabiRxByteInBits.bit_3 = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
  // For last 3 bits (6th, 7th and 8th) 
  for(bitIndex = 5; bitIndex <= 7; bitIndex++)
  {
    minimumTransitionSampleDistance = MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
    for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR); \
        receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR);\
        receivedSampleIndex++)
    {
      // Read sample values
      currentSampleValue = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
      previousSampleValue = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
      // Detect transition 
      if(currentSampleValue !=  previousSampleValue)
      {
        thisTransitionSampleIndex = receivedSampleIndex;
        thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
        // Detect minimum distance from default theoritical transition 
        if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
        {
          minimumTransitionSampleDistance = thisTransitionSampleDistance;
          minimumTransitionSampleIndex = thisTransitionSampleIndex;
        }
      }
    }
    // Adjust for next bit 
    lastTransitionSampleIndex = minimumTransitionSampleIndex;
    // Estimate this bit 
    switch(bitIndex)
    { 
      case 5:
        chabiRxCode[1].ChabiRxByteInBits.bit_2 = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 6:
        chabiRxCode[1].ChabiRxByteInBits.bit_1 = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 7:
        chabiRxCode[1].ChabiRxByteInBits.bit_0 = gManchesterSamplesByte2[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      default:
        break;
    }
  }
  // Estimate Byte # 2 Done ******************************************

  // Estimate Byte # 3 ******************************************
  chabiRxCode[2].ChabiRxByte = 0x00; // Init this byte with zeros
  lastTransitionSampleIndex = 0; // To be adjusted after each bit estimate
  // For 1st bit in this byte 
  bitIndex = 0;
  minimumTransitionSampleDistance = MAXIMUM_1ST_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
  for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR); \
      receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR);\
      receivedSampleIndex++)
  {
    // Read sample values
    currentSampleValue = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
    previousSampleValue = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
    // Detect transition 
    if(currentSampleValue !=  previousSampleValue)
    {
      thisTransitionSampleIndex = receivedSampleIndex;
      thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_1ST_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
      // Detect minimum distance from default theoritical transition 
      if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
      {
        minimumTransitionSampleDistance = thisTransitionSampleDistance;
        minimumTransitionSampleIndex = thisTransitionSampleIndex;
      }
    }
  }
  // Adjust for next bit 
  lastTransitionSampleIndex = minimumTransitionSampleIndex;
  // Estimate this bit 
  chabiRxCode[2].ChabiRxByteInBits.bit_7 = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
  // For next 3 bits (2nd, 3rd and 4th)
  for(bitIndex = 1; bitIndex <= 3; bitIndex++)
  {
    minimumTransitionSampleDistance = MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
    for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR); \
        receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR);\
        receivedSampleIndex++)
    {
      // Read sample values
      currentSampleValue = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
      previousSampleValue = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
      // Detect transition 
      if(currentSampleValue !=  previousSampleValue)
      {
        thisTransitionSampleIndex = receivedSampleIndex;
        thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
        // Detect minimum distance from default theoritical transition 
        if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
        {
          minimumTransitionSampleDistance = thisTransitionSampleDistance;
          minimumTransitionSampleIndex = thisTransitionSampleIndex;
        }
      }
    }
    // Adjust for next bit 
    lastTransitionSampleIndex = minimumTransitionSampleIndex;
    // Estimate this bit 
    switch(bitIndex)
    { 
      case 1:
        chabiRxCode[2].ChabiRxByteInBits.bit_6 = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 2:
        chabiRxCode[2].ChabiRxByteInBits.bit_5 = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 3:
        chabiRxCode[2].ChabiRxByteInBits.bit_4 = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      default:
        break;
    }
  }
  // For 5th bit in this byte 
  bitIndex = 4;
  minimumTransitionSampleDistance = MAXIMUM_5TH_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
  for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR); \
      receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR);\
      receivedSampleIndex++)
  {
    // Read sample values
    currentSampleValue = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
    previousSampleValue = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
    // Detect transition 
    if(currentSampleValue !=  previousSampleValue)
    {
      thisTransitionSampleIndex = receivedSampleIndex;
      thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_5TH_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
      // Detect minimum distance from default theoritical transition 
      if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
      {
        minimumTransitionSampleDistance = thisTransitionSampleDistance;
        minimumTransitionSampleIndex = thisTransitionSampleIndex;
      }
    }
  }
  // Adjust for next bit 
  lastTransitionSampleIndex = minimumTransitionSampleIndex;
  // Estimate this bit 
  chabiRxCode[2].ChabiRxByteInBits.bit_3 = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
  // For last 3 bits (6th, 7th and 8th) 
  for(bitIndex = 5; bitIndex <= 7; bitIndex++)
  {
    minimumTransitionSampleDistance = MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
    for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR); \
        receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR);\
        receivedSampleIndex++)
    {
      // Read sample values
      currentSampleValue = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
      previousSampleValue = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
      // Detect transition 
      if(currentSampleValue !=  previousSampleValue)
      {
        thisTransitionSampleIndex = receivedSampleIndex;
        thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
        // Detect minimum distance from default theoritical transition 
        if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
        {
          minimumTransitionSampleDistance = thisTransitionSampleDistance;
          minimumTransitionSampleIndex = thisTransitionSampleIndex;
        }
      }
    }
    // Adjust for next bit 
    lastTransitionSampleIndex = minimumTransitionSampleIndex;
    // Estimate this bit 
    switch(bitIndex)
    { 
      case 5:
        chabiRxCode[2].ChabiRxByteInBits.bit_2 = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 6:
        chabiRxCode[2].ChabiRxByteInBits.bit_1 = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 7:
        chabiRxCode[2].ChabiRxByteInBits.bit_0 = gManchesterSamplesByte3[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      default:
        break;
    }
  }  
  // Estimate Byte # 3 Done ******************************************
    
  // Estimate Byte # CRC ******************************************
  gReceivedCrcValueFromChabi.ChabiRxByte = 0x00; // Init this byte with zeros
  lastTransitionSampleIndex = 0; // To be adjusted after each bit estimate
  // For 1st bit in this byte 
  bitIndex = 0;
  minimumTransitionSampleDistance = MAXIMUM_1ST_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
  for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR); \
      receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_1ST_BIT_ESTIMATION_COUNT_IN_ISR);\
      receivedSampleIndex++)
  {
    // Read sample values
    currentSampleValue = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
    previousSampleValue = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
    // Detect transition 
    if(currentSampleValue !=  previousSampleValue)
    {
      thisTransitionSampleIndex = receivedSampleIndex;
      thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_1ST_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
      // Detect minimum distance from default theoritical transition 
      if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
      {
        minimumTransitionSampleDistance = thisTransitionSampleDistance;
        minimumTransitionSampleIndex = thisTransitionSampleIndex;
      }
    }
  }
  // Adjust for next bit 
  lastTransitionSampleIndex = minimumTransitionSampleIndex;
  // Estimate this bit 
  gReceivedCrcValueFromChabi.ChabiRxByteInBits.bit_7 = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
  // For next 3 bits (2nd, 3rd and 4th)
  for(bitIndex = 1; bitIndex <= 3; bitIndex++)
  {
    minimumTransitionSampleDistance = MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
    for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR); \
        receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR);\
        receivedSampleIndex++)
    {
      // Read sample values
      currentSampleValue = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
      previousSampleValue = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
      // Detect transition 
      if(currentSampleValue !=  previousSampleValue)
      {
        thisTransitionSampleIndex = receivedSampleIndex;
        thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
        // Detect minimum distance from default theoritical transition 
        if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
        {
          minimumTransitionSampleDistance = thisTransitionSampleDistance;
          minimumTransitionSampleIndex = thisTransitionSampleIndex;
        }
      }
    }
    // Adjust for next bit 
    lastTransitionSampleIndex = minimumTransitionSampleIndex;
    // Estimate this bit 
    switch(bitIndex)
    { 
      case 1:
        gReceivedCrcValueFromChabi.ChabiRxByteInBits.bit_6 = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 2:
        gReceivedCrcValueFromChabi.ChabiRxByteInBits.bit_5 = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 3:
        gReceivedCrcValueFromChabi.ChabiRxByteInBits.bit_4 = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      default:
        break;
    }
  }
  // For 5th bit in this byte 
  bitIndex = 4;
  minimumTransitionSampleDistance = MAXIMUM_5TH_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
  for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR); \
      receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_5TH_BIT_ESTIMATION_COUNT_IN_ISR);\
      receivedSampleIndex++)
  {
    // Read sample values
    currentSampleValue = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
    previousSampleValue = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
    // Detect transition 
    if(currentSampleValue !=  previousSampleValue)
    {
      thisTransitionSampleIndex = receivedSampleIndex;
      thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_5TH_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
      // Detect minimum distance from default theoritical transition 
      if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
      {
        minimumTransitionSampleDistance = thisTransitionSampleDistance;
        minimumTransitionSampleIndex = thisTransitionSampleIndex;
      }
    }
  }
  // Adjust for next bit 
  lastTransitionSampleIndex = minimumTransitionSampleIndex;
  // Estimate this bit 
  gReceivedCrcValueFromChabi.ChabiRxByteInBits.bit_3 = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
  // For last 3 bits (6th, 7th and 8th) 
  for(bitIndex = 5; bitIndex <= 7; bitIndex++)
  {
    minimumTransitionSampleDistance = MAXIMUM_BIT_TRANSITION_DISTANCE_COUNT_IN_ISR;
    for(receivedSampleIndex = (lastTransitionSampleIndex+START_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR); \
        receivedSampleIndex <= (lastTransitionSampleIndex+END_WINDOW_BIT_ESTIMATION_COUNT_IN_ISR);\
        receivedSampleIndex++)
    {
      // Read sample values
      currentSampleValue = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][receivedSampleIndex];
      previousSampleValue = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][receivedSampleIndex-1];
      // Detect transition 
      if(currentSampleValue !=  previousSampleValue)
      {
        thisTransitionSampleIndex = receivedSampleIndex;
        thisTransitionSampleDistance = abs(lastTransitionSampleIndex + DEFAULT_BIT_TRANSITION_CENTRE_COUNT_IN_ISR - thisTransitionSampleIndex);
        // Detect minimum distance from default theoritical transition 
        if(thisTransitionSampleDistance <= minimumTransitionSampleDistance)
        {
          minimumTransitionSampleDistance = thisTransitionSampleDistance;
          minimumTransitionSampleIndex = thisTransitionSampleIndex;
        }
      }
    }
    // Adjust for next bit 
    lastTransitionSampleIndex = minimumTransitionSampleIndex;
    // Estimate this bit 
    switch(bitIndex)
    { 
      case 5:
        gReceivedCrcValueFromChabi.ChabiRxByteInBits.bit_2 = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 6:
        gReceivedCrcValueFromChabi.ChabiRxByteInBits.bit_1 = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      case 7:
        gReceivedCrcValueFromChabi.ChabiRxByteInBits.bit_0 = gManchesterSamplesByteCrc[gManchesterSampleEstimateQueueIndex][lastTransitionSampleIndex-1];
        break;
      default:
        break;
    }
  }  
  // Estimate Byte # CRC Done ******************************************

  // Adjust Queindex
  gManchesterSampleEstimateQueueIndex++;
  if(gManchesterSampleEstimateQueueIndex >= NUMBER_OF_500us_SAMPLES_QUEUES)
  {
    gManchesterSampleEstimateQueueIndex = 0;
  }
}
