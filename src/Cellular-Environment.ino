/*
* Project Environmental Sensor - converged software for Low Power and Solar
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland chip@mcclellands.org
* Sponsor: Thom Harvey ID&D
* Date: 1 March 2019
*/

// v0.10 - Initial Release - BME680 functionality
// v1.01 - Added Boron Specific Signal strength data
// V1.02 - Added Adafruit STEMMA Soil Moisture Sensor
// v1.03 - Added webhook information for the soil sensor
// v1.04 - Adding watchdog Timer support from the Electron Carrier
// v1.05 - Fixed measurement bug
// v1.06 - Fixed Watchdog interrupt bug


#define SOFTWARERELEASENUMBER "1.06"               // Keep track of release numbers

// Included Libraries
#include "Adafruit_BME680.h"
#include "math.h"

namespace MEM_MAP {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x0,                    // Where we store the memory map version number - 8 Bits
    alertCountAddr        = 0x1,                    // Where we store our current alert count - 8 Bits
    resetCountAddr        = 0x2,                    // This is where we keep track of how often the Electron was reset - 8 Bits
    timeZoneAddr          = 0x3,                    // Store the local time zone data - 8 Bits
    controlRegisterAddr   = 0x4,                    // This is the control register for storing the current state - 8 Bits
    currentCountsTimeAddr = 0x5,                    // Time of last report - 32 bits
  };
};

#define SEALEVELPRESSURE_HPA (1013.25)              // Universal variables
#define MEMORYMAPVERSION 1                          // Lets us know if we need to reinitialize the memory map

Adafruit_BME680 bme;                                // Instantiate the I2C library

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);          // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);               // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;             // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                           // Initalize the PMIC class so you can call the Power Management functions below.


// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, MEASURING_STATE, SLEEPING_STATE, LOW_BATTERY_STATE, REPORTING_STATE};
State state = INITIALIZATION_STATE;

// Pin Constants
const int blueLED =       D7;                     // This LED is on the Electron itself
const int userSwitch =    D5;                     // User switch with a pull-up resistor
const int donePin =       D6;                     // This pin is used to let the watchdog timer know we are still alive
const int wakeUpPin =     A7;                     // Pin the watchdog will ping us on

volatile bool watchDogFlag = false;

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
int resetCount;                                     // Counts the number of times the Electron has had a pin reset
int alertCount;                                     // Keeps track of non-reset issues - think of it as an indication of health
bool ledState = LOW;                                // variable used to store the last LED status, to toggle the light
bool readyForBed = false;                           // Checks to see if steps for sleep have been completed
bool waiting = false;
bool doneEnabled = true;
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
byte controlRegister;                               // Stores the control register values
bool solarPowerMode;                                // Changes the PMIC settings
bool verboseMode;                                   // Enables more active communications for configutation and setup

// Variables Related To Particle Mobile Application Reporting
char SignalString[64];                     // Used to communicate Wireless RSSI and Description
const char* radioTech[8] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154"};
char TVOCString[16];                                // Simplifies reading values in the Particle Mobile Application
char temperatureString[16];
char humidityString[16];
char altitudeString[16];
char pressureString[16];
char heatIndexString[16];
char batteryString[16];

// Time Period Related Variables
time_t t;                                           // Global time vairable
byte currentHourlyPeriod;                           // This is where we will know if the period changed
byte currentDailyPeriod;                            // We will keep daily counts as well as period counts

// Battery monitoring
int stateOfCharge = 0;                              // Stores battery charge level value
int lowBattLimit;                                   // Trigger for Low Batt State
bool lowPowerMode;                                  // Flag for Low Power Mode operations

// This section is where we will initialize sensor specific variables, libraries and function prototypes
double temperatureInC = 0;
double relativeHumidity = 0;
double pressureHpa = 0;
double gasResistanceKOhms = 0;
double approxAltitudeInM = 0;
float heatIndexC;                                                 // Will cacluate the heat index and report to Ubidots


void setup()                                                      // Note: Disconnected Setup()
{
  char StartupMessage[64] = "Startup Successful";                 // Messages from Initialization
  state = IDLE_STATE;

  pinMode(blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output
  pinMode(userSwitch,INPUT);                                      // Momentary contact button on board for direct user input
  pinMode(donePin,OUTPUT);                                        // To pet the watchdog
  pinMode(wakeUpPin, INPUT);                                      // Watchdog interrrupt

  char responseTopic[125];
  String deviceID = System.deviceID();                            // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);  // Subscribe to the integration response event

  Particle.variable("Signal", SignalString);                      // Particle variables that enable monitoring using the mobile app
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", batteryString);
  Particle.variable("lowPowerMode",lowPowerMode);
  Particle.variable("temperature", temperatureString);
  Particle.variable("humidity", humidityString);
  Particle.variable("pressure", pressureString);
  Particle.variable("gas", TVOCString);
  Particle.variable("altitude", altitudeString);
  Particle.variable("Heat-Index",heatIndexString);

  Particle.function("Measure-Now",measureNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("SetTimeZone",setTimeZone);

  if (MEMORYMAPVERSION != EEPROM.read(MEM_MAP::versionAddr)) {          // Check to see if the memory map is the right version
    EEPROM.put(MEM_MAP::versionAddr,MEMORYMAPVERSION);
    for (int i=1; i < 10; i++) {
      EEPROM.put(i,0);                                                 // Zero out the memory - new map or new device
    }
  }

  if (!bme.begin()) {                                                   // Start the BME680 Sensor
    resetTimeStamp = millis();
    snprintf(StartupMessage,sizeof(StartupMessage),"Error - BME680 Initialization");
    state = ERROR_STATE;
  }


  // Set up the smapling paramatures
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  resetCount = EEPROM.read(MEM_MAP::resetCountAddr);                     // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET)                   // Check to see if we are starting from a pin reset
  {
    resetCount++;
    EEPROM.write(MEM_MAP::resetCountAddr, resetCount);                    // If so, store incremented number - watchdog must have done This
  }
  if (resetCount >=6) {                                                 // If we get to resetCount 4, we are resetting without entering the main loop
    EEPROM.write(MEM_MAP::resetCountAddr,4);                                           // The hope here is to get to the main loop and report a value of 4 which will indicate this issue is occuring
    fullModemReset();                                                   // This will reset the modem and the device will reboot
  }

  int8_t tempTimeZoneOffset = EEPROM.read(MEM_MAP::timeZoneAddr);                      // Load Time zone data from FRAM
  if (tempTimeZoneOffset <= 12 && tempTimeZoneOffset >= -12)  Time.zone((float)tempTimeZoneOffset);  // Load Timezone from FRAM
  else Time.zone(0);                                                   // Default is GMT in case proper value not in EEPROM

  // And set the flags from the control register
  controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);            // Read the Control Register for system modes so they stick even after reset
  lowPowerMode    = (0b00000001 & controlRegister);                     // Set the lowPowerMode
  solarPowerMode  = (0b00000100 & controlRegister);                     // Set the solarPowerMode
  verboseMode     = (0b00001000 & controlRegister);                     // Set the verboseMode

  PMICreset();                                                          // Executes commands that set up the PMIC for Solar charging - once we know the Solar Mode

  takeMeasurements();                                                   // For the benefit of monitoring the device

  if (!digitalRead(userSwitch)) {                                     // Rescue mode to locally take lowPowerMode so you can connect to device
  lowPowerMode = false;                                             // Press the user switch while resetting the device
    controlRegister = (0b11111110 & controlRegister);       // Turn off Low power mode
    EEPROM.write(controlRegister,MEM_MAP::controlRegisterAddr);         // Write to the EEMPROM
  }

  if (!lowPowerMode && (stateOfCharge >= lowBattLimit)) connectToParticle();  // If not lowpower or sleeping, we can connect
  connectToParticle();  // For now, let's just connect

  attachInterrupt(wakeUpPin,watchdogISR,RISING);

  if(verboseMode) Particle.publish("Startup",StartupMessage,PRIVATE);           // Let Particle know how the startup process went
  lastPublish = millis();
}

void loop()
{

  switch(state) {
  case IDLE_STATE:
    if (watchDogFlag) petWatchdog();
    if (!waiting && lowPowerMode && millis() > (keepAwakeTimeStamp+sleepWait)) state = SLEEPING_STATE;
    if (Time.hour() != currentHourlyPeriod) state = MEASURING_STATE;    // We want to report on the hour but not after bedtime
    if (stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;               // The battery is low - sleep
    break;

  case MEASURING_STATE:
    if (!takeMeasurements())
    {
      state = ERROR_STATE;
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Error taking Measurements",PRIVATE);
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
          Particle.publish("State","Going to Sleep",PRIVATE);
          lastPublish = millis();
        }
        delay(1000);                                                    // Time to send last update
        disconnectFromParticle();                                       // If connected, we need to disconned and power down the modem
      }
      EEPROM.write(MEM_MAP::resetCountAddr,resetCount);
      ledState = false;
      digitalWrite(blueLED,LOW);                                        // Turn off the LED
      readyForBed = true;                                               // Set the flag for the night
    }
    int secondsToHour = (60*(60 - Time.minute()));                      // Time till the top of the hour
    System.sleep(SLEEP_MODE_SOFTPOWEROFF,secondsToHour);                // Very deep sleep till the next hour - then resets
    } break;


  case LOW_BATTERY_STATE: {                                             // Sleep state but leaves the fuel gauge on
      if (Particle.connected()) {
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Low Battery - Sleeping",PRIVATE);
          lastPublish = millis();
        }
        delay(1000);                                                    // Time to send last update
        disconnectFromParticle();                                       // If connected, we need to disconned and power down the modem
      }
      ledState = false;
      digitalWrite(blueLED,LOW);                                        // Turn off the LED
      int secondsToHour = (60*(60 - Time.minute()));                    // Time till the top of the hour
      System.sleep(SLEEP_MODE_DEEP,secondsToHour);                      // Very deep sleep till the next hour - then resets
    } break;

  case REPORTING_STATE:                                                 // Reporting - hourly or on command
    if (!Particle.connected()) connectToParticle();
    else if (!waiting)
    {
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Reporting",PRIVATE,PRIVATE);
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
        Particle.publish("State","Idle",PRIVATE);
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
        Particle.publish("State","Error - Reporting Timed Out",PRIVATE);
        lastPublish = millis();
      }
    }
    break;

  case ERROR_STATE:                                          // To be enhanced - where we deal with errors
    if (millis() > resetTimeStamp + resetWait)
    {
      Particle.publish("State","ERROR_STATE - Resetting",PRIVATE);
      delay(2000);                                          // This makes sure it goes through before reset
      if (resetCount <= 3)  System.reset();                 // Today, only way out is reset
      else {
        EEPROM.write(MEM_MAP::resetCountAddr,0);            // Zero the ResetCount
        fullModemReset();                                   // Full Modem reset and reboot
      }
    }
    break;
  }
}

void sendEvent()
{
  char data[256];                                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"Temperature\":%4.1f, \"Humidity\":%4.1f, \"Pressure\":%4.1f, \"HeatIndex\":%4.1f, \"TVOClevel\":%5.1f, \"Altitude\":%4.1f, \"Battery\":%i, \"Resets\":%i, \"Alerts\":%i}", temperatureInC, relativeHumidity, pressureHpa, heatIndexC, gasResistanceKOhms,approxAltitudeInM, stateOfCharge,resetCount, alertCount);
  Particle.publish("Environmental_Hook", data, PRIVATE);
  currentHourlyPeriod = Time.hour();                                      // Change the time period
  currentDailyPeriod = Time.day();
  doneEnabled = false;
}

void UbidotsHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{temperature.0.status_code}}" so, I should only get a 3 digit number back
  char dataCopy[strlen(data)+1];                                    // data needs to be copied since Particle.publish() will clear it
  strncpy(dataCopy, data, sizeof(dataCopy));                        // Copy - overflow safe
  if (!strlen(dataCopy)) {                                          // First check to see if there is any data
    Particle.publish("Ubidots Hook", "No Data",PRIVATE);
    return;
  }
  int responseCode = atoi(dataCopy);                    // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    Particle.publish("State","Response Received",PRIVATE);
    doneEnabled = true;                                   // Successful response - can pet the dog again
  }
  else Particle.publish("Ubidots Hook", dataCopy,PRIVATE);       // Publish the response code
}

// These are the functions that are part of the takeMeasurements call

bool takeMeasurements() {

  bme.setGasHeater(320, 150); // 320*C for 150 ms
  bme.performReading();                                     // Take measurement from all the sensors

  if (Cellular.ready()) getSignalStrength();                // Test signal strength if the cellular modem is on and ready

  temperatureInC = bme.temperature;
  snprintf(temperatureString,sizeof(temperatureString),"%4.1f*C", temperatureInC);

  relativeHumidity = bme.humidity;
  snprintf(humidityString,sizeof(humidityString),"%4.1f%%", relativeHumidity);

  pressureHpa = bme.pressure / 100.0;
  snprintf(pressureString,sizeof(pressureString),"%4.1fHPa", pressureHpa);

  gasResistanceKOhms = bme.gas_resistance / 1000.0;
  snprintf(TVOCString,sizeof(TVOCString),"%4.1fkOhm", gasResistanceKOhms);

  approxAltitudeInM = bme.readAltitude(SEALEVELPRESSURE_HPA);
  snprintf(altitudeString,sizeof(altitudeString),"%4.1fm", approxAltitudeInM);

  stateOfCharge = int(batteryMonitor.getSoC());             // Percentage of full charge
  snprintf(batteryString,sizeof(batteryString),"%i%%", stateOfCharge);

  heatIndexC = heatIndex(temperatureInC,relativeHumidity);    // Calcualte the heat index when it is hot AND humid
  snprintf(heatIndexString,sizeof(heatIndexString),"%4.1f*C",heatIndexC);

  return 1;
}

void getSignalStrength()
{
  // New Boron capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}


// These functions control the connection and disconnection from Particle
bool connectToParticle() {
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {
    Particle.process();
  }
  if (Particle.connected()) return 1;                               // Were able to connect successfully
  else return 0;                                                    // Failed to connect
}

bool disconnectFromParticle()
{
  Particle.disconnect();                                          // Otherwise Electron will attempt to reconnect on wake
  Cellular.off();
  delay(1000);                                                    // Bummer but only should happen once an hour
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
float heatIndex(float TC, float R)
{
    float TF = (TC * 1.8) + 32.0;                // Need to convert to Farenheit for the calculations

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

    float simpleHeatIndex = ((c10 * ((TF+c11+(TF-c12)*c13) + (R*c14))) -32.0) / 1.8;

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
      return ((A + B + C - D + E) - 32.0) / 1.8;
    }
    else return simpleHeatIndex;

}

// These are the particle functions that allow you to configure and run the device
// They are intended to allow for customization and control during installations
// and to allow for management.


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
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b00000100 | controlRegister);          // Turn on solarPowerMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister);// Write it to the register
    PMICreset();                                               // Change the power management Settings
    Particle.publish("Mode","Set Solar Powered Mode",PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    solarPowerMode = false;
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b11111011 & controlRegister);           // Turn off solarPowerMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
    PMICreset();                                                // Change the power management settings
    Particle.publish("Mode","Cleared Solar Powered Mode",PRIVATE);
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b00001000 | controlRegister);                    // Turn on verboseMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
    Particle.publish("Mode","Set Verbose Mode",PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b11110111 & controlRegister);                    // Turn off verboseMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
    Particle.publish("Mode","Cleared Verbose Mode",PRIVATE);
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
  EEPROM.write(MEM_MAP::timeZoneAddr,tempTimeZoneOffset);                             // Store the new value in FRAMwrite8
  t = Time.now();
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  Particle.publish("Time",data,PRIVATE);
  delay(1000);
  Particle.publish("Time",Time.timeStr(t),PRIVATE);
  return 1;
}


int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    Particle.publish("Mode","Low Power",PRIVATE);
    controlRegister = (0b00000001 | controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = true;
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    Particle.publish("Mode","Normal Operations",PRIVATE);
    controlRegister = (0b1111110 & controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = false;
  }
  EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
  return 1;
}


bool meterParticlePublish(void)
{
  if(millis() - lastPublish >= publishFrequency) return 1;
  else return 0;
}

void fullModemReset() {  // Adapted form Rikkas7's https://github.com/rickkas7/electronsample

	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=16\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}

void watchdogISR() {
  watchDogFlag = true;
}

void petWatchdog() {
  digitalWrite(donePin,HIGH);
  digitalWrite(donePin,LOW);
  watchDogFlag = false;
}
