/*
* Project Environmental Sensor - converged software for Low Power and Solar
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland chip@mcclellands.org
* Sponsor: Simple Sense - alex@simplesense.io  www.simplesense.io
* Date: 21 May 2018
*/

// Note, requires a device with a >1 hour watchdog interval to work!

/*  The idea of this release is to unify the code base between PIR sensors
    Both implementations will move over to the finite state machine approach
    Both implementations will observe the park open and closing hours.
    I will also increase the ability for the end-user to configure the sensor without reflashing
    I will add two new states: 1) Low Power mode - maintains functionality but conserves battery by
    enabling sleep  2) Low Battery Mode - reduced functionality to preserve battery charge
    The watchdog timer should be set with a period of over 1 hour for the lowest power useage

    The mode and states will be set and recoded in the CONTROLREGISTER so resets will not change the mode
    Control Register - bits 7-4, 3 - Verbose Mode, 2- Solar Power Mode, 1 - Open, 0 - Low Power Mode
*/

// Easy place to change global numbers
//These defines let me change the memory map and configuration without hunting through the whole program
#define VERSIONNUMBER 9               // Increment this number each time the memory map is changed
#define WORDSIZE 8                    // For the Word size the number of bytes in a "word"
#define PAGESIZE 4096                 // Memory size in bytes / word size - 256kb FRAM
#define CURRENTOFFSET 24              // First word of hourly counts (remember we start counts at 1)
#define CURRENTCOUNTNUMBER 4064       // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
// First Word - 8 bytes for setting global values
#define VERSIONADDR 0x0               // Where we store the memory map version number
#define CURRENTHOURADDR 0x1           // This is how we avoid sending duplicate hourly reports
#define RESETCOUNT 0x2                // This is where we keep track of how often the Electron was reset
// Byte 0x3                           // Open
#define TIMEZONE  0x4                 // Store the local time zone data
// Bytes 0x5 & 0x6                    // Open
#define CONTROLREGISTER 0x7           // This is the control register for storing the current state - future use

// #define for the BME280 sensor
#define SEALEVELPRESSURE_HPA (1013.25)

// Finally, here are the variables I want to change often and pull them all together here
#define SOFTWARERELEASENUMBER "0.20"

// Included Libraries
#include "Adafruit_FRAM_I2C.h"        // Library for FRAM functions
#include "FRAM-Library-Extensions.h"  // Extends the FRAM Library
#include "electrondoc.h"              // Documents pinout
#include "Adafruit_BME280.h"
#include "Adafruit_CCS811.h"
#include "Adafruit_Sensor.h"

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);          // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);               // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;             // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                           // Initalize the PMIC class so you can call the Power Management functions below.
Adafruit_BME280 bme;                  // Protoype for the BME280 Sensor
Adafruit_CCS811 ccs;                  // Prototype for the CCS811 Sensor


// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, MEASURING_STATE, SLEEPING_STATE, LOW_BATTERY_STATE, REPORTING_STATE};
State state = INITIALIZATION_STATE;

// Pin Constants
const int tmp36Pin =      A0;                     // Simple Analog temperature sensor
const int wakeUpPin =     A7;                     // This is the Particle Electron WKP pin
const int tmp36Shutdwn =  B5;                     // Can turn off the TMP-36 to save energy
const int intPin =        D3;                     // CCS811 Sensor Interrupt pin
const int enablePin =     D2;                     // Turns on the CCS811 Sensor
const int hardResetPin =  D4;                     // Power Cycles the Electron and the Carrier Board
const int userSwitch =    D5;                     // User switch with a pull-up resistor
const int donePin =       D6;                     // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;                     // This LED is on the Electron itself

// Timing Variables
unsigned long webhookWaitTime = 45000;            // How long will we wair for a WebHook response
unsigned long publishFrequency = 1000;            // How often will we publish to Particle
unsigned long resetWait = 30000;                  // How long will we wait in ERROR_STATE until reset
unsigned long sleepWait = 90000;                  // How long do we wait until we go to sleep
unsigned long keepAwakeTimeStamp = 0;             // Starts the sleep clock
unsigned long resetTimeStamp = 0;                 // When did we start waiting to reset
unsigned long webhookTimeStamp = 0;               // Keep track of when we publish a webhook
unsigned long lastPublish = 0;                    // When was the last time we published


// Program Variables
int internalTemp;                                   // Global variable so we can monitor via cloud variable
int resetCount;                                     // Counts the number of times the Electron has had a pin reset
bool ledState = LOW;                                // variable used to store the last LED status, to toggle the light
bool readyForBed = false;                           // Checks to see if steps for sleep have been completed
bool waiting = false;
bool doneEnabled = true;
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
byte controlRegister;                               // Stores the control register values
bool solarPowerMode;                                // Changes the PMIC settings
bool verboseMode;                                   // Enables more active communications for configutation and setup
retained char Signal[17];                           // Used to communicate Wireless RSSI and Description
const char* levels[6] = {"Poor", "Low", "Medium", "Good", "Very Good", "Great"};
float heatIndexF;                                   // Will cacluate the heat index and report to Ubidots
char heatIndexString[16];

// Time Period Related Variables
time_t t;                                           // Global time vairable
byte currentHourlyPeriod;                           // This is where we will know if the period changed
byte currentDailyPeriod;                            // We will keep daily counts as well as period counts

// Battery monitoring
int stateOfCharge = 0;                              // Stores battery charge level value
int lowBattLimit;                                   // Trigger for Low Batt State
bool lowPowerMode;                                  // Flag for Low Power Mode operations


// This section is where we will initialize sensor specific variables, libraries and function prototypes
// CCS811 Air Quality Sensor variables
float ccsCO2;
float ccsTVOC;
char ccsCO2String[16];
char ccsTVOCString[16];

// BME280 Temperature / Humidity / Barometric Pressure Sensor
float bmeTemp;
float bmePressure;
// float bmeAltitude;
float bmeHumidity;
char bmeTempString[8];
char bmePressureString[16];
char bmeHumidityString[8];

void setup()                                // Note: Disconnected Setup()
{
  char StartupMessage[64] = "Startup Successful"; // Messages from Initialization
  state = IDLE_STATE;

  pinMode(enablePin,OUTPUT);                // For GPS enabled units
  digitalWrite(enablePin,LOW);              // Turn on the CCS811 Sensor
  pinMode(wakeUpPin,INPUT);                 // This pin is active HIGH
  pinMode(userSwitch,INPUT);                // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                 // declare the Blue LED Pin as an output
  pinMode(tmp36Shutdwn,OUTPUT);             // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);         // Turns on the temp sensor
  pinMode(donePin,OUTPUT);                  // Allows us to pet the watchdog
  pinMode(hardResetPin,OUTPUT);             // For a hard reset active HIGH

  watchdogISR();                            // Pet the watchdog

  char responseTopic[125];
  String deviceID = System.deviceID();                                // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("Signal", Signal);                                // Particle variables that enable monitoring using the mobile app
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerMode);
  Particle.variable("CO2level",ccsCO2String);
  Particle.variable("TVOClevel",ccsTVOCString);
  Particle.variable("Temperature",bmeTempString);
  Particle.variable("Barometric",bmePressureString);
  Particle.variable("Humidity",bmeHumidityString);
  Particle.variable("Heat-Index",heatIndexString);

  Particle.function("Reset-FRAM", resetFRAM);                         // These functions allow you to configure and control the Electron
  Particle.function("Hard-Reset",hardResetNow);
  Particle.function("Measure-Now",measureNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("SetTimeZone",setTimeZone);

  Wire.begin();                                                         // Create a waire object

  if (!fram.begin()) {                                                  // You can stick the new i2c addr in here, e.g. begin(0x51);
    resetTimeStamp = millis();
    snprintf(StartupMessage,sizeof(StartupMessage),"Error - FRAM Initialization");
    state = ERROR_STATE;
  }
  else if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {                   // Check to see if the memory map in the sketch matches the data on the chip
    ResetFRAM();                                                        // Reset the FRAM to correct the issue
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {
      resetTimeStamp = millis();
      snprintf(StartupMessage,sizeof(StartupMessage),"Error - FRAM Write Error");
      state = ERROR_STATE;
    }
    else {
      FRAMwrite8(CONTROLREGISTER,0);                                    // Need to reset so not in low power or low battery mode
    }
  }

  if (!bme.begin(0x76)) {                                               // Start the BME280 Sensor
    resetTimeStamp = millis();
    snprintf(StartupMessage,sizeof(StartupMessage),"Error - BME280 Initialization");
    state = ERROR_STATE;
  }

  if(!ccs.begin()) {                                                    // Start the CCS811 Sensor
    resetTimeStamp = millis();
    snprintf(StartupMessage,sizeof(StartupMessage),"Error - CCS811 Initialization");
    state = ERROR_STATE;
  }

  resetCount = FRAMread8(RESETCOUNT);                                   // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET)                   // Check to see if we are starting from a pin reset
  {
    resetCount++;
    FRAMwrite8(RESETCOUNT,static_cast<uint8_t>(resetCount));            // If so, store incremented number - watchdog must have done This
  }

  int8_t tempTimeZoneOffset = FRAMread8(TIMEZONE);                      // Load Time zone data from FRAM
  if (tempTimeZoneOffset <= 12 && tempTimeZoneOffset >= -12)  Time.zone((float)tempTimeZoneOffset);  // Load Timezone from FRAM
  else Time.zone(-5);                                                   // Default is EST in case proper value not in FRAM

  // And set the flags from the control register
  controlRegister = FRAMread8(CONTROLREGISTER);                         // Read the Control Register for system modes so they stick even after reset
  lowPowerMode    = (0b00000001 & controlRegister);                     // Set the lowPowerMode
  solarPowerMode  = (0b00000100 & controlRegister);                     // Set the solarPowerMode
  verboseMode     = (0b00001000 & controlRegister);                     // Set the verboseMode

  if(!digitalRead(userSwitch)) {
    snprintf(StartupMessage,sizeof(StartupMessage),"User Button - Normal Power Mode");
    controlRegister = (0b1111110 & controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = false;
    FRAMwrite8(CONTROLREGISTER,controlRegister);
  }

  PMICreset();                                                          // Executes commands that set up the PMIC for Solar charging - once we know the Solar Mode

  delay(1000);                                                           // make sure the CCS811 sensor is ready
  takeMeasurements();                                                   // For the benefit of monitoring the device

  if (!lowPowerMode && (stateOfCharge >= lowBattLimit)) connectToParticle();  // If not lowpower or sleeping, we can connect
  currentHourlyPeriod = FRAMread8(CURRENTHOURADDR);                     // Sets the hour period for when the count starts (see #defines)

  attachInterrupt(wakeUpPin, watchdogISR, RISING);                      // The watchdog timer will signal us and we have to respond

  if(verboseMode) Particle.publish("Startup",StartupMessage);           // Let Particle know how the startup process went
  lastPublish = millis();

  // digitalWrite(enablePin,HIGH);                                         // Turn off the CCS811 Sensor
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (!waiting && millis() > (keepAwakeTimeStamp+sleepWait)) state = SLEEPING_STATE;
    if (Time.hour() != currentHourlyPeriod) state = MEASURING_STATE;    // We want to report on the hour but not after bedtime
    if (stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;               // The battery is low - sleep
    break;

  case MEASURING_STATE:
    if (!takeMeasurements())
    {
      state = ERROR_STATE;
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Error taking Measurements");
        lastPublish = millis();
      }
    }
    else state = REPORTING_STATE;
    break;

  case SLEEPING_STATE: {                                                // This state is triggered once the park closes and runs until it opens
    if (!readyForBed)                                                   // Only do these things once - at bedtime
    {
      if (Particle.connected()) {
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Going to Sleep");
          lastPublish = millis();
        }
        delay(1000);                                                    // Time to send last update
        disconnectFromParticle();                                       // If connected, we need to disconned and power down the modem
      }
      FRAMwrite8(RESETCOUNT,resetCount);
      ledState = false;
      digitalWrite(blueLED,LOW);                                        // Turn off the LED
      digitalWrite(tmp36Shutdwn, LOW);                                  // Turns off the temp sensor
      // digitalWrite(enablePin,HIGH);                                      // Turns off the CCS811 Sensor
      watchdogISR();                                                    // Pet the watchdog
      readyForBed = true;                                               // Set the flag for the night
    }
    int secondsToHour = (60*(60 - Time.minute()));                      // Time till the top of the hour
    System.sleep(SLEEP_MODE_SOFTPOWEROFF,secondsToHour);                // Very deep sleep till the next hour - then resets
    } break;


  case LOW_BATTERY_STATE: {                                             // Sleep state but leaves the fuel gauge on
      if (Particle.connected()) {
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Low Battery - Sleeping");
          lastPublish = millis();
        }
        delay(1000);                                                    // Time to send last update
        disconnectFromParticle();                                       // If connected, we need to disconned and power down the modem
      }
      ledState = false;
      digitalWrite(blueLED,LOW);                                        // Turn off the LED
      digitalWrite(tmp36Shutdwn, LOW);                                  // Turns off the temp sensor
      digitalWrite(enablePin,HIGH);                                     // Turns off the CCS811 Sensor
      watchdogISR();                                                    // Pet the watchdog
      int secondsToHour = (60*(60 - Time.minute()));                    // Time till the top of the hour
      System.sleep(SLEEP_MODE_DEEP,secondsToHour);                      // Very deep sleep till the next hour - then resets
    } break;

  case REPORTING_STATE:                                                 // Reporting - hourly or on command
    if (!Particle.connected()) connectToParticle();
    else if (!waiting)
    {
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Reporting");
        lastPublish = millis();
      }
      webhookTimeStamp = millis();
      waiting = true;                                                   // Make sure we set the flag for flow through this case
      sendEvent();                                                      // Send the data to Ubidots
    }
    else if (waiting && doneEnabled)
    {
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Idle");
        lastPublish = millis();
      }
      state = IDLE_STATE;       // This is how we know if Ubidots got the data
      waiting = false;
      keepAwakeTimeStamp = millis();
    }
    else if (waiting && millis() >= (webhookTimeStamp + webhookWaitTime))
    {
      state = ERROR_STATE;
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Error - Reporting Timed Out");
        lastPublish = millis();
      }
    }
    break;

  case ERROR_STATE:                                          // To be enhanced - where we deal with errors
    if (millis() > resetTimeStamp + resetWait)
    {
      Particle.publish("State","ERROR_STATE - Resetting");
      delay(2000);                                          // This makes sure it goes through before reset
      if (resetCount <= 3)  System.reset();                 // Today, only way out is reset
      else {
        FRAMwrite8(RESETCOUNT,0);                           // Time for a hard reset
        digitalWrite(hardResetPin,HIGH);                    // Zero the count so only every three
      }
    }
    break;
  }
}

void sendEvent()
{
  char data[256];                                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"Temperature\":%4.1f, \"Humidity\":%4.1f, \"Pressure\":%4.1f, \"HeatIndex\":%4.1f, \"CO2level\": %5.1f, \"TVOClevel\":%5.1f, \"Battery\":%i, \"Resets\":%i}",bmeTemp,bmeHumidity,bmePressure,heatIndexF,ccsCO2,ccsTVOC,stateOfCharge,resetCount);
  Particle.publish("Environmental_Hook", data, PRIVATE);
  currentHourlyPeriod = Time.hour();                                      // Change the time period
  FRAMwrite8(CURRENTHOURADDR,currentHourlyPeriod);                        // Write to FRAM
  currentDailyPeriod = Time.day();
  doneEnabled = false;
}

void UbidotsHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{temperature.0.status_code}}" so, I should only get a 3 digit number back
  char dataCopy[strlen(data)+1];                                    // data needs to be copied since Particle.publish() will clear it
  strncpy(dataCopy, data, sizeof(dataCopy));                        // Copy - overflow safe
  if (!strlen(dataCopy)) {                                          // First check to see if there is any data
    Particle.publish("Ubidots Hook", "No Data");
    return;
  }
  int responseCode = atoi(dataCopy);                    // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    Particle.publish("State","Response Received");
    doneEnabled = true;                                   // Successful response - can pet the dog again
    watchdogISR();                                        // Pet the watchdog - just in case we missed an interrupt
  }
  else Particle.publish("Ubidots Hook", dataCopy);       // Publish the response code
}

// These are the functions that are part of the takeMeasurements call

bool takeMeasurements() {
  if (Cellular.ready()) getSignalStrength();                // Test signal strength if the cellular modem is on and ready
  getTemperature();                                         // Get Temperature from the onboard sensor
  stateOfCharge = int(batteryMonitor.getSoC());             // Percentage of full charge

  //digitalWrite(enablePin,LOW);                              // Turn on the CCS811 Sensor
  delay(250);

  while(!ccs.available());                                  // Calibrate the CCS Sensor
  float ccsTemp = ccs.calculateTemperature();
  ccs.setTempOffset(ccsTemp - 25.0);

  while(!ccs.available());                                  // After reading temp, make sure ready for next read
  if(!ccs.readData())
  {
    ccsCO2 = ccs.geteCO2();
    snprintf(ccsCO2String,sizeof(ccsCO2String),"%5.1fppm",ccsCO2);

    ccsTVOC = ccs.getTVOC();
    snprintf(ccsTVOCString,sizeof(ccsTVOCString),"%5.1fppb",ccsTVOC);
  }
  else return 0;

  bmeTemp = bme.readTemperature()*1.8+32.0;
  snprintf(bmeTempString,sizeof(bmeTempString),"%4.1f*F", bmeTemp);

  bmePressure = bme.readPressure() / 100.0F;
  snprintf(bmePressureString,sizeof(bmePressureString),"%4.1fhPa", bmePressure);

  bmeHumidity = bme.readHumidity();
  snprintf(bmeHumidityString,sizeof(bmeHumidityString),"%4.1f%%", bmeHumidity);

  // bmeAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);               // Not using this for now

  heatIndexF = heatIndex(bmeTemp,bmeHumidity);    // Calcualte the heat index when it is hot AND humid
  snprintf(heatIndexString,sizeof(heatIndexString),"%4.1f*F",heatIndexF);

  return 1;
}

void getSignalStrength()
{
    CellularSignal sig = Cellular.RSSI();  // Prototype for Cellular Signal Montoring
    int rssi = sig.rssi;
    int strength = map(rssi, -131, -51, 0, 5);
    snprintf(Signal,17, "%s: %d", levels[strength], rssi);
}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                    // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  internalTemp = int(temperatureC * 1.8 + 32.0);  // now convert to Fahrenheit
  return internalTemp;
}


void watchdogISR()
{
  if (doneEnabled) {
    digitalWrite(donePin, HIGH);                      // Pet the watchdog
    digitalWrite(donePin, LOW);
  }
}

// These functions control the connection and disconnection from Particle
bool connectToParticle()
{
  if (!Cellular.ready())
  {
    Cellular.on();                                           // turn on the Modem
    Cellular.connect();                                      // Connect to the cellular network
    if(!waitFor(Cellular.ready,90000)) return false;         // Connect to cellular - give it 90 seconds
  }
  Particle.process();
  Particle.connect();                                      // Connect to Particle
  if(!waitFor(Particle.connected,30000)) return false;     // Connect to Particle - give it 30 seconds
  Particle.process();
  return true;
}

bool disconnectFromParticle()
{
  Particle.disconnect();                                   // Disconnect from Particle in prep for sleep
  waitFor(notConnected,10000);
  Cellular.disconnect();                                   // Disconnect from the cellular network
  delay(3000);
  Cellular.off();                                           // Turn off the cellular modem
  return true;
}

bool notConnected() {
  return !Particle.connected();                             // This is a requirement to use waitFor
}

// Power Management function
void PMICreset() {
  power.begin();                                            // Settings for Solar powered power management
  power.disableWatchdog();
  if (solarPowerMode) {
    lowBattLimit = 20;                                      // Trigger for Low Batt State
    power.setInputVoltageLimit(4840);                       // Set the lowest input voltage to 4.84 volts best setting for 6V solar panels
    power.setInputCurrentLimit(900);                        // default is 900mA
    power.setChargeCurrent(0,0,1,0,0,0);                    // default is 512mA matches my 3W panel
    power.setChargeVoltage(4208);                           // Allows us to charge cloe to 100% - battery can't go over 45 celcius
  }
  else  {
    lowBattLimit = 30;                                      // Trigger for Low Batt State
    power.setInputVoltageLimit(4208);                       // This is the default value for the Electron
    power.setInputCurrentLimit(1500);                       // default is 900mA this let's me charge faster
    power.setChargeCurrent(0,1,1,0,0,0);                    // default is 2048mA (011000) = 512mA+1024mA+512mA)
    power.setChargeVoltage(4112);                           // default is 4.112V termination voltage
  }
}

// This function calculates the Heat Index According to this Formula
//  http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
// I have tested the results against this table and found this to be within 1%
// https://www.weather.gov/safety/heat-index
// TF = temp in F  -  R = humidity in %
// My starting point was: https://github.com/RobTillaart/Arduino/blob/master/libraries/Temperature/temperature.h
float heatIndex(float TF, float R)
{
    // Constants for the Rothfusz regression - valid over 80F
    const float c1 = -42.379;
    const float c2 =  2.04901523;
    const float c3 = 10.14333127;
    const float c4 = -0.22475541;
    const float c5 = -0.00683783;
    const float c6 = -0.05481717;
    const float c7 =  0.00122874;
    const float c8 =  0.00085282;
    const float c9 = -0.00000199;
    // Constants for the high heat and low humidity adjustment (hl or high/low)
    const float hl1 = 13.0;
    const float hl2 = 4.0;
    const float hl3 = 17.0;
    const float hl4 = 95.0;
    float D = 0;
    // Constants for the high heat and high humidity adjustment (hh or high/high)
    const float hh1 = 85.0;
    const float hh2 = 10.0;
    const float hh3 = 87.0;
    const float hh4 = 5.0;
    float E = 0;

    // Constants for a simplified regression valid below 80F
    const float c10 = 0.5;
    const float c11 = 61.0;
    const float c12 = 68.0;
    const float c13 = 1.2;
    const float c14 = 0.094;

    float simpleHeatIndex = c10 * ((TF+c11+(TF-c12)*c13) + (R*c14));

    if (((TF+simpleHeatIndex)/2) > 80.0) {
      float A = (( c5 * TF) + c2) * TF + c1;
      float B = (((c7 * TF) + c4) * TF + c3) * R;
      float C = (((c9 * TF) + c8) * TF + c6) * R * R;
      if (TF > 80 && TF < 112 && R < 13) {   // Low humidity adjustment
        D = ((hl1-R)/hl2)*sqrt((hl3-abs(TF-hl4))/17);
      }
      if (TF > 80 && TF < 87 && R > 85) {  // High humidity adjustment
        E = ((R-hh1)/hh2)*((hh3-TF)/hh4);
      }
      return A + B + C - D + E;
    }
    else return simpleHeatIndex;

}

// These are the particle functions that allow you to configure and run the device
// They are intended to allow for customization and control during installations
// and to allow for management.

int resetFRAM(String command)   // Will reset the local counts
{
  if (command == "1")
  {
    ResetFRAM();
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)   // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    digitalWrite(hardResetPin,HIGH);          // This will cut all power to the Electron AND the carrir board
    return 1;                                 // Unfortunately, this will never be sent
  }
  else return 0;
}

int measureNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = MEASURING_STATE;
    return 1;
  }
  else return 0;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    solarPowerMode = true;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b00000100 | controlRegister);          // Turn on solarPowerMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);               // Write it to the register
    PMICreset();                                               // Change the power management Settings
    Particle.publish("Mode","Set Solar Powered Mode");
    return 1;
  }
  else if (command == "0")
  {
    solarPowerMode = false;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b11111011 & controlRegister);           // Turn off solarPowerMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                // Write it to the register
    PMICreset();                                                // Change the power management settings
    Particle.publish("Mode","Cleared Solar Powered Mode");
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b00001000 | controlRegister);                    // Turn on verboseMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                        // Write it to the register
    Particle.publish("Mode","Set Verbose Mode");
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b11110111 & controlRegister);                    // Turn off verboseMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                        // Write it to the register
    Particle.publish("Mode","Cleared Verbose Mode");
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  FRAMwrite8(TIMEZONE,tempTimeZoneOffset);                             // Store the new value in FRAMwrite8
  t = Time.now();
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  Particle.publish("Time",data);
  delay(1000);
  Particle.publish("Time",Time.timeStr(t));
  return 1;
}


int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  controlRegister = FRAMread8(CONTROLREGISTER);                       // Get the control register (generla approach)
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    Particle.publish("Mode","Low Power");
    controlRegister = (0b00000001 | controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = true;
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    Particle.publish("Mode","Normal Operations");
    controlRegister = (0b1111110 & controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = false;
  }
  FRAMwrite8(CONTROLREGISTER,controlRegister);                         // Write to the control register
  return 1;
}


bool meterParticlePublish(void)
{
  if(millis() - lastPublish >= publishFrequency) return 1;
  else return 0;
}
