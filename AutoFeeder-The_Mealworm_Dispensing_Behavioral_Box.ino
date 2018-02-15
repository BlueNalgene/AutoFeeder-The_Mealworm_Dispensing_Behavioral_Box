/*
 *  Automatic Birdfeeder Script
 *  
 *  Wesley T. Honeycutt
 *  
 *    Code in current version is modified from ETAG RFID Reader from AOS2017
 * 
 * ---------------------------------------------Previous Version----------------------------------------
 *  Data logging sketch for the ETAG RFID Reader
 *  Version 1. Distributed at AOS2017
 *   
 *   Code by:   
 *   Alexander Moreno   
 *   David Mitchell   
 *   Eli Bridge   
 *   July-2017
 *    
 *    Licenced in the public domain
 *     
 *     REQUIREMENTS:  
 *     The files "logger.cpp" and "logger.h" must be in the same folder as this sketch file
 *     Power supply for the board should come from the USB cable or a 5V battery or DV power source.  
 *     A 3.3V CR1025 coin battery should be installed on the back side of the board to maintain the date      
 *     and time when the board is disconnected from a primary power source.
 */

//INITIALIZE INCLUDE FILES AND I/O PINS
#include <Servo.h>              //include library to automatically deal with servo PWM.
#include "logger.h"             //include the library for reading and parsing raw RFID input
#include <Wire.h>               //include the standard wire library - used for I2C communication with the clock
#include <SD.h>                 //include the standard SD card library
#define Serial SerialUSB        //Designate the USB connection as the primary serial comm port
#define DEMOD_OUT_PIN   30      //(PB03) this is the target pin for the raw RFID data
#define SHD_PINA         8      //(PA06) Setting this pin high activates the primary RFID circuit (only one active at a time)
//#define SHD_PINB         9    //(PA07) Setting this pin high activates the seconday RFID circuit (only one active at a time)
#define MOD_PIN          0      //not used - defined as zero
#define READY_CLOCK_PIN  0      //not used - defined as zero
#define chipSelect       7      //Chip Select for SD card - make this pin low to activate the SD card
#define LED_RFID        31      //Pin to control the LED indicator.  
logger L(DEMOD_OUT_PIN, MOD_PIN, READY_CLOCK_PIN); //Designate pins used in the logger library

// initialize variables
byte tagData[5];                       //Variable for storing RFID tag codes
unsigned long currentMillis;           //Used for exploiting the ms counter for timing functions 
unsigned long stopMillis;              //Used for exploiting the ms counter for timing functions 
byte RFcircuit = 1;                    //Used to determine which RFID circuit is active. 1 = primary, 2 = secondary. 
unsigned int serDelay;                 //Used for timing on initiation of serial communication
byte ss, mm, hh, da, mo, yr;           //Byte variables for storing date/time elements
String sss, mms, hhs, das, mos, yrs;   //String variable for storing date/time text elements
String timeString;                     //String for storing the whole date/time line of data
byte incomingByte = 0;                 //Used for incoming serial data
unsigned int timeIn[12];               //Used for incoming serial data during clock setting

Servo myservo;                         // Initialize servo library.

// MODE TOGGLE INTEGER
int modeToggle           = 0;          // This toggle can be values from 0 to 3.  It tells the code to execute certain modes:
                                       //
                                       // 0 - Feeds whatever lands on the perch.
                                       // 1 - This feeder will only feed birds with tags
                                       // 2 - This feeder will only feed birds with tags AND pass the color learning test.
                                       // 3 - This feeder will only feed birds with tags AND bird unique tests

// IMPORTANT Addressing Bits
const int nodeAddress    = 0;          // 0 is Master, otherwise Slave
const int maxNodes       = 1;          // Number of nodes.  Include the Master.

// Do you have the above code correct?

// Color Assignments
const int degColor1      = 30;         // red on color spinner (see diagram)
const int degColor2      = 90;         // white on color spinner (see diagram)
const int degColor3      = 120;        // green on color spinner (see diagram)

// Pin Assignments
const int wormSwitch     = 6;          // Mechanical mylar switch to detect falling worm.
const int servoTrans     = 10;         // Transistor enables the servo
const int leverPin       = 11;         // This is the perch of the birdfeeder.
const int doorSwitch     = 12;         // Switch detects door open
const int feederSwitch   = 13;         // Switches at top and bottom of plunger are attached here.
const int feederMotor1   = 2;          // TB6612FNG In1 to servo breakout board.
const int feederMotor2   = 3;          // TB6612FNG In2 to servo breakout board.
const int feederMotor3   = 4;          // TB6612FNG PWM to servo breakout board.
const int servoPin       = A2;         // Output to data line of servo.  Requires PWM

// Volatile variables
volatile byte wormState  = 0;          // This is the worm detection bit. Now a physical switch.

// Regular variables
int correctFeeder;                     // Used to select feeder which is the correct answer
byte statusFlag          = 1;          // This int is used by the Master to determine how to administer tasks
                                       // Different values set different status flags
                                       // 0 == Default.  All is well/Nothing to report.
                                       // 1 == Unloaded. This unit fed a bird recently. Starts up Unloaded.
                                       // 2 == Fail.     A bird made a mistake at this feeder.
byte flags[maxNodes];                  // Finally, we tell the Master to make an empty array of bytes for later.

// Logging Parameter Constants
const unsigned int polltime = 3000;    //How long in milliseconds to poll for tags 
const unsigned int pausetime = 500;    //How long in milliseconds to wait between polling intervals 
const unsigned int readFreq = 200;     //How long to wait after a tag is successfully read.

// Variables for a simple speed test  // DEBUG
byte clockState = LOW; // DEBUG
int clockLine = 2; // DEBUG

 /*****************************************    
  *     
  *            Spinner Design
  * angles represent middle of pie sections
  *  for a half circle.  Adjust for other 
  *        color-wheel geometries.
  *        
  *                 90°
  *             ____________
  *            /\          /\
  *           /  \   w    /  \ 
  *      30° /    \      /    \ 120°
  *         /  r   \    /  g   \
  *         |       \  /       |
  *     0°  | _______\/_______ | 180°
  *         |                  |
  *         |                  |
  *         \                 /
  *          \               /
  *           \             /
  *            \___________/
  *            
  *****************************************/





void setup() {  // This function sets everything up for logging.
  pinMode(SHD_PINA, OUTPUT);     // Make the primary RFID shutdown pin an output.
  digitalWrite(SHD_PINA, LOW);   // turn the primary RFID circuit off (HIGH turns it on)
  pinMode(LED_RFID, OUTPUT);
  digitalWrite(LED_RFID, HIGH);  // turn the LED off (LOW turns it on)
  initclk();                     // Calls a function to start the clock ticking if it wasn't doing so already

  //Try to initiate a serial connection
  Serial.begin(9600);               //Initiate a serial connection with the given baud rate
  ss = 0;                           //Initialize a variable for counting in the while loop that follows
  while (ss < 5 && !Serial) {       //flash LED 5 times while waiting for serial connection to come online
    delay(500);                     //pause for 0.5 seconds
    digitalWrite(LED_RFID, LOW);    //turn the LED on (LOW turns it on)
    delay(500);                     //pause again
    digitalWrite(LED_RFID, HIGH);   //turn the LED off (HIGH turns it off)
    ss = ss + 1;                    //add to counting variable
  }                                 //end while
  digitalWrite(LED_RFID, HIGH);     // make sure LED is off 

  // Get your motor running.
  pinMode(feederMotor1, OUTPUT); //init our motor
  pinMode(feederMotor2, OUTPUT);
  pinMode(feederMotor3, OUTPUT);
  motorOff(); //make sure our motor is stopped

  // Use internal pullups for switch lines.  This must be changed a two-line pinMode/digitalWrite if ported to other boards.
  pinMode(wormSwitch, INPUT_PULLUP);    // Note that these switches are pulled up, inverted logic...
  pinMode(feederSwitch, INPUT_PULLUP);  // So when the circuit is closed and electricity is going through the switch...
  pinMode(leverPin, INPUT_PULLUP);      // it will be in the LOW state.  Once the switch is not activated, the circuit...
  pinMode(doorSwitch, INPUT_PULLUP);    // is open, and the switch is in the HIGH state.

  // Interrupts are declared
  attachInterrupt(wormSwitch, worm_ISR, CHANGE); //Any CHANGE on pin wormSwitch will instantly interrupt with code in worm_ISR

  // Interrupt volatile byte is set here.  Presumably the switch is not closed during setup.
  wormState = digitalRead(wormSwitch);

  // The servo library is told to do work on servoPin
  myservo.attach(servoPin);

  // The transistor to switch the servo on and off to save energy is declared.
  pinMode(servoTrans, OUTPUT);


  // Now we need to set up communications between the nodes of our network (I2C has a Master and one or more Slaves)
  // The section of code below represents a simple setup of I2C and confirmation that each unit knows its ID.

  //This starts the Master else Slave nodes.
  if (nodeAddress == 0) {    // if you are a Master
    Wire.begin();            // start your communications port
    delay(600);              // and wait for everyone else to catch up
  }
  else {                     // otherwise, you are a Slave
    delay(30);               // hold on a moment while Master does it's thing
    Wire.begin(nodeAddress); // then turn on your communications port
  }

  //If the node is a Master, request 6 bytes from each Slave
  //If the node is a Slave, respond to the request to say you a ready.
  if (nodeAddress == 0 ) {                 // if this node is a Master
    for (int i = 1; i <= maxNodes; i++) {  // for each unit in our network (declared by maxNodes)
      Wire.requestFrom(i, 6);              // request 6 bytes.
      while(Wire.available()) {            // Listen for the bytes to come in.
        char c = Wire.read();              // when they bytes come in, treat them as characters
        Serial.print(c);                   // and print those characters to the serial port for debugging
        delay(60);                         // then wait a moment
        //Wes, add logging of working units later!
      }
    }
  }
  else {                                   // otherwise, this unit is a Slave
    Wire.onRequest(startupRequest);        // so when the Master asks, serve him what is in startupRequest
    delay(60);                             // then wait a moment
  }
  // This concludes the I2C setup section

  // This section allows the user to set the time on the real time clock
  Serial.println("set clock?  Y or n.");             //Ask if the user wants to set the clock
  serDelay = 0;                                      //If there's no response then just start logging
  while(Serial.available() == 0 && serDelay < 10000) //wait about 10 seconds for a user response
    {
      delay(1);                                     
      serDelay++;
    } 
    if (Serial.available()){           //If there is a response check to see if it is "Y"
      incomingByte = Serial.read();
    } 
    if(incomingByte == 89) {           //"Y" is the ascii character equivalent of 89
      setClk();                        //if the user enters "Y" (89) then call the clock setting function
    }
    
  //Set up the SD card
  Serial.print("Initializing SD card...\n");              //message to user
  if (!SD.begin(chipSelect)) {                            //Initiate the SD card function with the pin that activates the card.
    Serial.println("\nSD card failed, or not present");   //SD card error message
    return;
  }// end check SD

  // Begin a cycle to release anything which is in the chamber and ensure position of plunger
  while (digitalRead(doorSwitch) == HIGH) {  // While the plunger is not opening the door
    motorCCW();                              // move the motor such that it will open the door
  }                                          // once the doorSwitch says the door is open
  motorBrake();                              // Brake yourself before you break yourself
  delay(60);                                 // Give that a moment to sink in.
  motorOff();                                // And power down the motor once it has stopped (why waste energy on brakes?)
  delay(1000);                               // Give anything in the chamber a second to roll out
  while (digitalRead(feederSwitch) == LOW) { // When the door is open like that, the feederSwitch is going nuts. 
    motorCW();                               // while it is busy going nuts, roll that plunger back up
  }                                          // until the feederSwitch stops making a racket.
  motorBrake();                              // Then brake.
  delay(60);                                 // let that sink in.
  motorOff();                                // And power down.  The plunger should be in just the right position
  delay(300);                                // this delay is not strictly necessary, but nice as a segue

} // end setup 


       

void loop() {  //This is the main function. It loops (repeats) forever.

  checkStatus();
  
  // PLUNGER AT REST

  wormLoader();
  
  

  if (modeToggle == 1 || modeToggle == 2) {     // If the mode is 1 OR 2
    colorTest();                                // The color wheel test is performed
  }                                             // 
  else if (modeToggle == 3) {                   // If the mode is 3
    individualTest();                           // A special variant of the color wheel test is called
  }                                             // 
  else if (modeToggle == 0) {                   // If the mode is 0
  }                                             // continue loop for a dumb feeder.
  else {                                        // If the mode toggle is not properly set
    Serial.print("ERROR: modeToggle = ");       // Report a debugging error to the user
    Serial.println(modeToggle);                 // This is what you set it as
    Serial.println("   only 0-3 are valid");    // This is the range you should have used
    Serial.println("Running as RFID Feeder");   // RFID only (mode 1). Feed birds.
  }
  
  
  // Now we have the worm loaded, and the color set and wait for our bird to arrive.
     
  while (leverPin == HIGH) {                 // While this switch is closed, nothing is happening
    delay(1000);                             // We perform a short delay.
    statusFlag = 1;                          // We tell the status that a worm is loaded, but there is nothing to report.
    return;                                  // And then we completely exit the void loop()
  }                                          // 

  // EVENT - BIRD LANDS AT THIS FEEDER
  Serial.println("BIRD!");  //  DEBUG
  digitalWrite(SHD_PINA, HIGH);              //we activate RFID and begin scanning here.
                                             // NOTE that we only are looking at a single RFID pin. 

  if (modeToggle != 0) {                       // If this is NOT a dumb feeder, we need to set up RFID
                                               //scan for a tag - if a tag is sucesfully scanned, return a 'true' and proceed
    currentMillis = millis();                  //To determine how long to poll for tags, first get the current 
                                               //value of the built in millisecond clock on the processor
    stopMillis = currentMillis + polltime;     //next add the value of polltime to the current clock time to 
                                               //determine the desired stop time.
    while(stopMillis > millis()) {             //As long as the stoptime is less than the current millisecond 
                                               //counter, then keep looking for a tag
      if (L.scanForTag(tagData) == true) {     //If a tag gets read, then do all the following stuff 
                                               //(if not it will keep trying until the timer runs out)
        if (correctFeeder == nodeAddress) {    // If the bird landed at the "correct" feeder...
          logEvent(1);                         // Log this event with a 1 then begin feeding
          statusFlag = 1;                      // Set the flag saying a bird was fed.
        }                                      
        else {                                 // If the bird landed on the "wrong" feeder...
          logEvent(0);                         // Log this event with a zero then ....
          statusFlag = 2;                      // Set the flag saying a bird failed a test.
          //////////////////////////////// tbc
          return;
          
        }
      }                                        // end millis countdown
    }                                          // end modeToggle != 0
    digitalWrite(SHD_PINA, LOW);                     //Turn off both RFID circuits
    delay(pausetime);                                //pause between polling attempts  
  }                                                  // end of modeToggle if statement
  while (digitalRead(doorSwitch) == HIGH) {          // While the trap door is not open
    motorCCW();                                      // Send the plunger down to open the door.
  }                                                  // And once it is down....
  motorBrake();                                      // STOP
  delay(30);                                         // Let that sink in
  motorOff();                                        // Stop the motor completely
  delay(1000);                                       // Wait 1 second...
  while (digitalRead(feederSwitch) == LOW) {         // While the plunger is down such that feederSwitch says it is not at rest
    motorCW();                                       // move the plunger up
  }                                                  // When it hits the button
  motorBrake();                                      // STOP
  delay(30);                                         // Let that sink in
  motorOff();                                        // Stop the motor completely

  // PLUNGER AT REST



          
                                                                                       
     

  
      
 
  delay(5);

  // Wes's little clock checker // DEBUG
  clockState =! clockState; // DEBUG
  digitalWrite(clockLine, clockState); // DEBUG
}   // end void loop













// Wes addition - functions

void worm_ISR() {
  wormState =! wormState;
  delay(500);
}

void servoMove(int deg) {
  digitalWrite(servoTrans, HIGH);
  delay(300);
  myservo.write(deg);
  delay(3000);  
  digitalWrite(servoTrans, LOW);  
}

void startupRequest() {
  Wire.write("ready ");
}

void motorOff() {
  digitalWrite(feederMotor1, LOW);
  digitalWrite(feederMotor2, LOW);
  digitalWrite(feederMotor3, LOW);
}

void motorCW() {
  digitalWrite(feederMotor1, LOW);
  digitalWrite(feederMotor2, HIGH);
  digitalWrite(feederMotor3, HIGH);
  delay(5);
}

void motorCCW() {
  digitalWrite(feederMotor1, HIGH);
  digitalWrite(feederMotor2, LOW);
  digitalWrite(feederMotor3, HIGH);
  delay(5);
}

void motorBrake() {
  digitalWrite(feederMotor1, HIGH);
  digitalWrite(feederMotor2, HIGH);
  digitalWrite(feederMotor3, LOW);
}

void colorTWI(int numBytes) {              // This tells a Slave the "correct" unit.
  correctFeeder = Wire.read();             // Whatever the Master tells you over I2C is the answer
  if (correctFeeder == nodeAddress) {      // If your node number is the "correct" answer...
    servoMove(degColor1);                  // Then turn your indicator to green.
  }
  else {                                   // If your node number was NOT the "correct" answer...
    servoMove(degColor3);                  // turn your indicator to red.
  }
}                                          // Now the Slaves are displaying the correct indication for the test.

void logEvent(byte correctChoice) {
  getTime();                               //Call a subroutine function that reads the time from the clock
  File dataFile = SD.open("datalog.txt", FILE_WRITE);        //Initialize the SD card and open the file or create if not there.
    if (dataFile) {                              
      for (int n = 0; n < 5; n++) {               //loop to print out the RFID code to the SD card
        if (tagData[n] < 10) dataFile.print("0"); //add a leading zero if necessary
        dataFile.print(tagData[n], HEX);          //print to the SD card
        }
      dataFile.print(",");                        //comma for data delineation
      dataFile.print(RFcircuit);                  //log which antenna was active
      dataFile.print(",");                        //comma for data delineation
      dataFile.print(timeString);                 //log the time
      dataFile.print(",");                        //comma for... you know the rest
      dataFile.println(correctChoice);            //log whether the bird made the correct choice or not.
      dataFile.close();                           //close the file
    } // check dataFile is present
    else {
      //Serial.println("error opening datalog.txt");  //error message if the "datafile.txt" is not present or cannot be created
    }// end check for file
}

void flagReport(int statusFlag) {  // placeholder for now.  Figure out what is going on with the INT.
}

void colorTest() {
  // Now we can begin our Test
  // The Master will assign one of the nodes to be "correct" node for the test
  if (nodeAddress == 0) {                       // If you are the Master
    correctFeeder = random(0, (maxNodes - 1) ); // pick the feeder which will feed at random from all of the nodes
    for (int i = 1; i <= maxNodes; i++) {       // For each node in the network...
      Wire.beginTransmission(i);                // tell each one...
      Wire.write(correctFeeder);                // which unit will be the "correct" test answer
      Wire.endTransmission();                   // Close the communication line after telling each Slave this info.
    }
    Serial.print("correctFeeder = ");  // DEBUG
    Serial.println(correctFeeder);  // DEBUG
    delay(60);                                 // And wait a moment after you are done.
    if (correctFeeder == nodeAddress) {        // If the Master was the "correct" feeder
      servoMove(degColor1);                    // Move the color wheel to the green indicator
      Serial.println("I'm Green"); // DEBUG
    }
    else {                                     // If the Master was NOT the "correct" feeder
      servoMove(degColor3);                    // Move the color wheel to the red indicator
      Serial.println("I'm Red"); // DEBUG
    }
  }                                            // This ends the Master section
  else {                                       // If you were not the Master, you must be a Slave
    Wire.onReceive(colorTWI);                  // Listen for input from the Master, then execute colorTWI
    delay(300);                                // Give the I2C a chance to communicate with the others.
  }
}

void individualTest() {
  // Placeholder for bird-specific tests.  TBD
}

void checkStatus() {  // In this function, the Master asks each Slave what its status is.  Then acts on it.
  for(int i = 0; i <= maxNodes;  ++i) {         // First we clean up our flag array...
    flags[i] = (byte)0;                         // by making sure everything is a 0
  }
  // Master Reader - Slave Sender
  if (nodeAddress == 0) {                       // If this unit is the Master
    flags[0] = statusFlag;                      // Then we put the Master's status in the array before the rest.
    for (int i = 1; i <= maxNodes; i++) {       // For each node in the network...
      Wire.requestFrom(i, 1);                   // ask each Slave for 1 byte...
      while(Wire.available()) {                 // while each Slave communicates the requested byte...
        flags[i] = Wire.read();                 // we add that byte to the flag array.
      }                                         //
    }                                           // Once that is taken care of, we need to analyze what we have.
  }                                             //
  else {                                        // Otherwise, this unit is a Slave
    Wire.onReceive(flagReport);                 // Listen for input from the Master, then execute flagReport
    delay(300);                                 // Give the I2C a chance to communicate with the others.
  }

  // Master Writer - Slave Receiver
  if (nodeAddress == 0) {                       // If this unit is the Master
    for(int i = 1; i <= maxNodes;  ++i) {       // For each node in the network...
      Wire.beginTransmission(i);                // send to that unit...
      Wire.write(flags, maxNodes);              // the array
      Wire.endTransmission();                   // Then stop
    }                                           //
  }                                             //
  else {                                        // Otherwise, this unit is a Slave
    receiveFlags(maxNodes);                     // The slave receives the flag array.
  }                                             //
  delay(10);                                    // Everybody does the delay and what follows
  for (int i = 0; i <= sizeof(flags); i++) {    // For each flag in the array:
            // Current Flag Identities:
            // 0 - No change/all is well
            // 1 - This unit fed a bird
            // 2 - A bird failed a test
    if (flags[i] == 1 && i == nodeAddress) {    // If the value is 1 AND also your address
      wormLoader();                             // Reload
    }                                           // Nice and simple
    else if (flags[i] == 1 && i != nodeAddress) { // If a bird got fed, but it isn't your problem
      delay(60);                                // Delay a bit.
    }
    else if (flags[i] == 2) {                   // If a bird just failed on a test
      servoMove(degColor2);                     // Turn the wheel to white
      delay(10000);                             // Wait 10 seconds
      statusFlag = 0;                           // And prepare to go again.
    }                                           //
  }                                             //
}                                               //

void wormLoader() {
  // This function loads a worm if the unit has not registered a worm in the chamber
  while (wormState == HIGH) {                   //only executes if we don't have a worm in the feeder
    while (digitalRead(feederSwitch) == HIGH) { // Until the plunger hits the top switch
      motorCW();                                // move the plunger to the top.
    }                                           // once it hits to top...
    motorBrake();                               // Stop.
    delay(30);                                  // let that sink in
    motorOff();                                 // power down the motor
    delay(4000);                                // Give anything picked up by the motor time to roll into the chamber
                                                // If nothing has hit the wormSwitch, then the wormState will still be HIGH.
    // PLUNGER AT TOP                           // If that is the case, while loop needs to go back to the bottom
                                                // to make another pass
    motorCCW();                                 // Send the motor back down
    delay(300);                                 // Give the plunger time to stop touching the top switch.
    while (digitalRead(feederSwitch) == HIGH) { // Until the plunger hits the bottom switch
      motorCCW();                               // continue moving down
    }  // PLUNGER AT REST                       // Once it hits the bottom switch...
    motorBrake();                               // stop
    delay(60);                                  // Wait a moment before starting this while loop again.
  }                                             // Or exit if you have the worm.

  // PLUNGER AT TOP                             // When we exit the previous loop, we expect it will be after dumping a worm.
                                                // So we need to take it back down to the reset position
  motorCCW();                                   // Send the motor back down
  delay(300);                                   // Give the plunger time to stop touching the top switch.
  while (digitalRead(feederSwitch) == HIGH) {   // Until the plunger hits the bottom switch
    motorCCW();                                 // continue moving down
  }                                             // Once it hits the bottom switch...
  motorBrake();                                 // stop
  delay(30);                                    // Let that sink in
  motorOff();                                   // Then cut power to the motor.
  statusFlag = 0;                               // Report that you have successfully loaded a worm.
  delay(30);                                    // And let that sink in before continuing the test.

  // PLUNGER AT REST 
  Serial.println("happydance"); // DEBUG
}

void receiveFlags(int byteLength) {             // This is the function for Slaves to receive flags
  while (1 < Wire.available()) {                // While the Wire is available
    char c = Wire.read();                       // Read each byte (aka char in Arduino C)....
    for (int i = 0; i <= byteLength;  ++i) {    // and each time you read one of these
      flags[i] = c;                             // put it in the empty flags array at the correct spot.
    }                                           // until we reach the end of the info.
  }                                             // Then the while ends.
  for (int i = 0; i <= maxNodes;  ++i) { // DEBUG
    Serial.println(flags[i]); // DEBUG
  }
}








////The Following are all subroutines called by the code above.//////////////

byte bcdToDec(byte val)  {   // Convert binary coded decimals (from the clock) to normal decimal numbers
  return ( (val/16*10) + (val%16) );
}

byte decToBcd( byte val ) {  // Convert decimal to binary coded decimals for writing to the clock
   return (byte) ((val / 10 * 16) + (val % 10));
}

static uint8_t conv2d(const char* p) { // Convert parts of a string to decimal numbers (not used in this version)
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

void initclk () {                //Start the clock running if it is not already
  Wire.begin();                  //Start up the I2C comm funcitons
  Wire.beginTransmission(0x68);  //Send the clock Slave address
  Wire.write(0x0C);              //address for clearing the HT (halt time?) bit
  Wire.write(0x3F);              //HT is bit 6 so 00111111 clears it.
  Wire.endTransmission();        //End the I2C transmission
}

void setClk() {                          //Function to set the clock)
  Serial.println("Enter mmddyyhhmmss");  //Ask for user input
   while(Serial.available() == 0) {}     //wait for 12 characters to accumulate
   for (int n = 0; n < 13; n++) {        //loop to read all the data from the serial buffer once it is ready
      timeIn[n] = Serial.read();         //Read the characters from the buffer into an array of 12 bytes one at a time
   }
    while(Serial.available())            //Clear the buffer, in case there were extra characters
      {Serial.read();}

   mo = ((timeIn[0]-48)*10 + (timeIn[1]-48));    //Convert two ascii characters into a single decimal number
   da = ((timeIn[2]-48)*10 + (timeIn[3]-48));    //Convert two ascii characters into a single decimal number
   yr = ((timeIn[4]-48)*10 + (timeIn[5]-48));    //Convert two ascii characters into a single decimal number
   hh = ((timeIn[6]-48)*10 + (timeIn[7]-48));    //Convert two ascii characters into a single decimal number       
   mm = ((timeIn[8]-48)*10 + (timeIn[9]-48));    //Convert two ascii characters into a single decimal number
   ss = ((timeIn[10]-48)*10 + (timeIn[11]-48));  //Convert two ascii characters into a single decimal number

  //Write the new time to the clock usind I2C protocols implemented in the Wire library
  initclk();                    //Make sure the clock is running
  Wire.beginTransmission(0x68); //Begin I2C communication using the I2C address for the clock
  Wire.write(0x00);             //starting register - register 0
  Wire.write(0x00);             //write to register 0 - psecs (100ths of a second - can only be set to zero
  Wire.write(decToBcd(ss));     //write to register 1 - seconds
  Wire.write(decToBcd(mm));     //write to register 2 - minutes
  Wire.write(decToBcd(hh));     //write to register 3 - hours 
  Wire.write(0x01);             //write to register 4 - day of the week (we don't care about this)
  Wire.write(decToBcd(da));     //write to register 5 - day of month
  Wire.write(decToBcd(mo));     //write to register 6 - month
  Wire.write(decToBcd(yr));     //write to register 7 - year
  Wire.endTransmission();
    
  getTime();                      //Read from the time registers you just set 
  Serial.print("Clock set to ");  //message confirming clock time
  Serial.println(timeString);

  //When the clock is set (more specifically when Serial.read is used) the RFID circuit fails)
  //I don't know why this happens
  //Only solution seems to be to restart the device. 
  //So the following messages inform the user to restart the device.
  Serial.print("Restart reader to log data."); 
  while(1){}                                     //Endless while loop. Program ends here. User must restart.
}

void getTime() {  //Read in the time from the clock registers
  Wire.beginTransmission(0x68);  //I2C address for the clock
  Wire.write(0x01);              //start to read from register 1 (seconds)
  Wire.endTransmission();
  Wire.requestFrom(0x68, 7);     //Request seven bytes from seven consecutive registers on the clock.
  if (Wire.available()) {
    ss = Wire.read(); //second
    mm = Wire.read(); //minute
    hh = Wire.read(); //hour
    da = Wire.read(); //day of week, which we don't care about. this byte gets overwritten in next line
    da = Wire.read(); //day of month
    mo = Wire.read(); //month
    yr = Wire.read(); //year
  }
  sss = ss < 10 ? "0"+String(bcdToDec(ss), DEC) : String(bcdToDec(ss), DEC); //These lines convert decimals to characters to build a
  mms = mm < 10 ? "0"+String(bcdToDec(mm), DEC) : String(bcdToDec(mm), DEC); //string with the date and time 
  hhs = hh < 10 ? "0"+String(bcdToDec(hh), DEC) : String(bcdToDec(hh), DEC); //They use a shorthand if/then/else statement to add 
  das = da < 10 ? "0"+String(bcdToDec(da), DEC) : String(bcdToDec(da), DEC); //leading zeros if necessary
  mos = mo < 10 ? "0"+String(bcdToDec(mo), DEC) : String(bcdToDec(mo), DEC);
  yrs = yr < 10 ? "0"+String(bcdToDec(yr), DEC) : String(bcdToDec(yr), DEC);
  timeString = mos+"/"+das+"/"+yrs+" "+hhs+":"+mms+":"+sss;                  //Construct the date and time string
}

