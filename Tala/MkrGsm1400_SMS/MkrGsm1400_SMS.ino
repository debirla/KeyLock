/*
 SMS receiver

 This sketch, for the MKR GSM 1400 board, waits for a SMS message
 and displays it through the Serial port.

 Circuit:
 * MKR GSM 1400 board
 * Antenna
 * SIM card that can receive SMS messages

 created 25 Feb 2012
 by Javier Zorzano / TD
*/

// include the GSM library
#include <MKRGSM.h>

// #include "arduino_secrets.h" 
// Please enter your sensitive data in the Secret tab or arduino_secrets.h
// PIN Number
const char PINNUMBER[] = "1413";
//#define PINNUMBER "1413"

// initialize the library instances
GSM gsmAccess;
GSM_SMS sms;

// Array to hold the number a SMS is retreived from
char senderNumber[20];
#define SMS_ACKN_TXT "Hi, yor message received in MKR GSM 1400 #"
unsigned char gSmsMsgCount = 0;
String smsAcknToSend;

void setup() 
{
  // initialize serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("SMS Messages Receiver and Send Acknowledge");

  // connection state
  bool connected = false;

  // Start GSM connection
  while (!connected) 
  {
    if (gsmAccess.begin(PINNUMBER) == GSM_READY) 
    {
      connected = true;
    } 
    else 
    {
      Serial.println("Not connected");
      delay(1000);
    }
  }

  Serial.println("GSM initialized");
  Serial.println("Waiting for messages");
}

void loop() 
{
  int c;
  

  // If there are any SMSs available()
  if (sms.available()) 
  {
    Serial.print ("Message received from: ");

    // Get remote number
    sms.remoteNumber(senderNumber, 20);
    if(String(senderNumber) == String("+919830724896"))
    {
      Serial.print("Mr Debojyoti Lahiri ");
      Serial.println(senderNumber);
    }
    else
    {
      Serial.println(senderNumber);
    }

    // An example of message disposal
    // Any messages starting with # should be discarded
    if (sms.peek() == '#') 
    {
      Serial.println("Discarded SMS");
      sms.flush();
    }

    // Read message bytes and print them
    while ((c = sms.read()) != -1) 
    {
      Serial.print((char)c);
    }

    Serial.println("\nEND OF MESSAGE");

    // Delete message from modem memory
    sms.flush();
    Serial.println("MESSAGE DELETED");

    // Send Ackn Message 
    smsAcknToSend = SMS_ACKN_TXT;
    gSmsMsgCount++;
    sms.beginSMS(senderNumber);
    smsAcknToSend += String(gSmsMsgCount);
    sms.print(smsAcknToSend);
    sms.endSMS();
    Serial.print("SMS SEND ACKN COMPLETE => SMS Number = ");
    Serial.println(String(gSmsMsgCount));
    delay(10000); // wait for ten seconds to avoid multiple messages
  }
  delay(1000);
}
