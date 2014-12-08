// BLUETOOTH commands for the BlueSmirf SILVER v.2
// NOTE: this is  Class 1, 100 mW (20 dBm), approx ~100 meters. 
// SAFETY: Much more safe than a cell phone! (approx 0.001 watts per kilogram, while a cellphone reaches 0.25W/kg. And note that
// the U.S. and Canadian governments have set a maximum SAR of 1.6 watts per kilogram, while the European Union permits a slightly higher level.) 
// Uses AT commands: BGB203 AT Command Set (http://www.sparkfun.com/datasheets/Wireless/Bluetooth/BGB203_SPP_UserGuide.pdf)

char remoteAddress[13] = "0806080E290D";

// Attempt to connect: 
void BTConnectCLIENT() {
// REM: AT+BTBDA returns local address 
// REM: depending on which is this program (the one which will attempt connection or not), set the bluetooth to accept incomming connections
// port.write("AT+BTSRV=1\r");
  
  Serial.print("+++"); // escape sequence (just in case): 
  delay(500);
   Serial.print("AT+BTCLT=");
   Serial.print(remoteAddress);
   Serial.print(", 1\r");
   delay(250);
}

void BTSetSERVER() {
// REM: AT+BTBDA returns local address 
// REM: depending on which is this program (the one which will attempt connection or not), set the bluetooth to accept incomming connections
// port.write("AT+BTSRV=1\r");
  
  Serial.print("+++"); // escape sequence (just in case). REM: no need to add <CR>
  delay(500);
    Serial.print("AT+BTAUT=0,1\r"); // for some reason autoconnect reset to 1 (auto) and then "CONNECT" does not appears!!!???
    delay(250);
   Serial.print("AT+BTSRV=1\r");
}

// Configure bluetooth for the very first time: 
void BTSetup() { //For using BlueSmirf SILVER v.2
  Serial.print("+++"); // escape sequence (just in case): 
  delay(500);
  
  // (1) Configure baud rate: (rem: this MUST be done beforehand, and save the values in the flash memory, otherwise it would be
  // impossible to connect to the serial-usb port...)
  // AT+BTURT=115200,8,0,1,0
   // REM: other baud rates are for the bluetooth module are: 9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800,921600 and 1000000 bps 
  // REM: looks like Arduino Pro Mini and Wee does not support >= 115200 baud...
  
  // We can dissable ECHO when in command mode (if we want - but we will have the "OKs" and also the "ATE0" command itself)
  // THIS WON'T AFFECT the "CONNECT" or "NO CARRIER" echo from DATA MODE!!!
     Serial.print("ATE0\r");
    delay(250);
  // (2) not automatic mode and not suppressed responses (to get information back about connection/disconnection states):
   Serial.print("AT+BTAUT=0,1\r");
   delay(250);
   Serial.print("AT+BTCFG=32\r");
   delay(250);
     // (4) link timeout (MAKE IT SHORT to avoid getting stuck when program stops...):
   Serial.print("AT+BTLSV=3"); //from 2 seconds to 40 seconds
    delay(250);
    
  // Finally, store in flash memory just in case (no need if we re-call to BTSetup each time): 
  Serial.print("AT+BTFLS\r");
  delay(2000);
  
  // In these conditions, if the connection is established, the module will answer "CONNECT XXXX", and when connection is lost,  "NO CARRIER"
}

// REM: other modules: 
/* old bluesmirf: 
void BTConnect() {
  Serial.print("+++\r");
  delay(250);
  Serial.print("ATDH\r");
  Serial.print("ATDM");
  Serial.print(remoteAddress);
  Serial.print(",1101\r");
}
*/


void initializeBluetooth() {
Serial.print("+++\r");
delay(250);
Serial.print("AT&F\r");
Serial.print("AT+BTLNM=\"HOULALED\"\r");
Serial.print("AT+BTAUT=1, 0\r");
Serial.print("AT+BTURT=115200, 8, 0, 1, 0\r"); // SET THE BLUETOOTH UART SPEED!
Serial.print("AT+BTSEC=0\r");
Serial.print("AT+BTFLS\r");
Serial.print("AT+BTSRV=1\r");
}

/* ------------------- NOTES -----------

This script:

  - resets to factory settings (so we know what state we're in)
  - changes the Bluetooth display name to 'somename'
  - allows automatic Bluetooth connections to the module
  - sets the module to 115200bps (at which point you will probably have to change the bit rate on your UART as well)
  - disables security so you don't need a complex pairing process (naughty, but it makes prototyping a whole lot easier)
  - writes all of this to the Flash on the module
  - starts the Bluetooth server
  
  (FROM : http://www.ianhowson.com/bluesmirf-silver-intro.html )
  
 ----------------------------------------  */
