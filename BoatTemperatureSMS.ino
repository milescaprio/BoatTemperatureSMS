//V1.2.0 8/7/2022 In progress
//changing chars to const char* s for warnings
//made function to check text
//got rid of \ in dweet

//20201115 PLAN dweet Tmin Tmax, remove DHT-T DHT-H
//?add seperate dweet delay
//future - reset min and max every 24 hours
//set auto alarm text
//update text to same data as dweet
//update loop periods
//add AIN for voltage and variables for Vmin Vnow Vmax
//CHECK loop times since they add and time will creep longer, use millis() mod and difference
//make dweet increment, loop increment, text increment changeable with incoming msgs.
//make arduino reset with incoming msgs.
//fix help menu and interface.
//reduce code to uno?
//clean code, check up, optimize!
//RELEASE INTO WILD!!

#include <dht_nonblocking.h>
#include <Adafruit_FONA.h>
#include <SoftwareSerial.h>

#define DHT_SENSOR_TYPE DHT_TYPE_11
#define SIMCOM_7000

#define DHT_PIN 50 //unused

// For SIM7000 shield
#define FONA_PWRKEY 6
#define FONA_RST 7
#define FONA_DTR 8 // Connect with solder jumper
#define FONA_RI 9 // Need to enable via AT commands
#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX
#define T_ALERT 12 // Connect with solder jumper

#define INCOMING_MSG_LIMIT 160

#define THERMISTORPIN A0
#define THERMISTORPOWER 2  // use this Digital output to supply 5v to thermistor so it can be cycled off between measurements

#define BILGEPIN 4
#define BILGEPOWER A5 // use this Digital output to supply 5v to bilge float switch\
so it can be cycled off between measurements. Run after thermistor to avoid drawing too many amps

#define VOLTMEASUREPIN A1 //with a 5.7 to 1 voltage divider!
#define VOLTDIVKOHM 47.0
#define VOLTPULLDOWNKOHM 10.0

#define ADMIN_NUMBER "PUT NUMBER 1 HERE"
#define WHITELISTED_1 "NUMBER 2 HERE"
#define WHITELISTED_2 "NUMBER 3 HERE"
#define URL_STRING "dweet.io/dweet/for/YOUR_DWEET_NAME_HERE?tempMin=%i.%i&tempNow=%i.%i&tempMax=%i.%i&voltMin=%i.%i&voltNow=%i.%i&voltMax=%i.%i&bilgeNow=%i&bilgeMax=%i&latitude=%i.%i%i&longitude=%i.%i%i"

#define millisrolloverfix(num) ((num) < 0 ? (num) + UNSIGNEDLONGARDUINO : (num))
#define timestampoverflowfix(num) ((num) >= UNSIGNEDLONGARDUINO ? (num) - UNSIGNEDLONGARDUINO : (num))
#define UNSIGNEDLONGARDUINO 4294967296

#define CHANGESTATETIME 10
#define LEDBLINKTIME 100

unsigned long loopPeriod = 60000;
unsigned long textIncrement = 86400000;
unsigned long dweetIncrement = 1800000;
unsigned long alarmRepeat = 3600000;

// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)
// Notice how we don't include the reset pin because it's reserved for emergencies on the LTE module!
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!

unsigned long lastTextMillis = millis();
unsigned long lastDweetMillis = millis();
unsigned long lastAlarmMillis = millis();
unsigned long lastLoopMillis = millis();
bool alarmTimerType = 0; //This value will be 0 when using the default 60-minute alarm repeat, but will be 1 when using a texted !DELAYALARM. Is reset after each alarm.
unsigned long userAlarmDelay = 0;

float tempMax = 0;
float tempMin = 255;
float tempCurrent = 0;
int tempAlarmMin = 35;
float voltAlarmMin = 11.5;
double thermTempMax = 0;
double thermTempMin = 255;
double thermTempCurrent = 0;
float humidCurrent = 0;
bool bilgeCurrent = 0;
bool bilgeMax = 0;
float voltCurrent = 0;
float voltMax = 0;
float voltMin = 255;
bool doSubscribe = false;

char lastTexter[15] = ADMIN_NUMBER;
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
DHT_nonblocking dht_sensor(DHT_PIN, DHT_SENSOR_TYPE);

void powerOn();
//--------------------------------SETUP--------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  //use for blinking during measure
  pinMode(THERMISTORPOWER, OUTPUT);    //
  digitalWrite(THERMISTORPOWER, LOW);  //
  pinMode(BILGEPOWER, OUTPUT);
  digitalWrite(BILGEPOWER, LOW);
  pinMode(BILGEPIN, INPUT);
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state
  pinMode(FONA_PWRKEY, OUTPUT);
  powerOn();
  Serial.begin(9600);
  Serial.println("ACTIVE VERSION: 1.1.0");
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take several seconds)"));

  // SIM7000 takes about 3s to turn on but SIM7500 takes about 15s
  // Press reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset

  fonaSS.begin(115200); // Default SIM7000 shield baud rate
  Serial.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  //delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (!fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    while (1); // Don't proceed if it couldn't find the device
  }
  switch (fona.type()) {
    case SIM7000A:
      Serial.println(F("SIM7000A (American)")); break;
    default:
      Serial.println(F("???"));
      Serial.println(F("Type of SIM module is not SIM7000A.")); break;
  }
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1
  //fona.setNetworkSettings(F("your APN"), F("your username"), F("your password"));
  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card
  //fona.setNetworkSettings(F("fast.t-mobile.com")); // For Tmobile SIM card

  //--------------------------------ONE-TIME--------------------------------

  if (!fona.enableGPRS(true)) Serial.println(F("Failed to turn on GPRS"));
  if (!fona.enableGPS(true)) Serial.println(F("Failed to turn on GPS"));
  delay(5000);
  // read the network/cellular status
  uint8_t networkStatus = fona.getNetworkStatus();
  Serial.print(F("Network status "));
  Serial.print(networkStatus);
  Serial.print(F(": "));
  if (networkStatus == 0) Serial.println(F("Not registered"));
  if (networkStatus == 1) Serial.println(F("Registered (home)"));
  if (networkStatus == 2) Serial.println(F("Not registered (searching)"));
  if (networkStatus == 3) Serial.println(F("Denied"));
  if (networkStatus == 4) Serial.println(F("Unknown"));
  if (networkStatus == 5) Serial.println(F("Registered roaming"));

  uint8_t ss = fona.getRSSI();
  int8_t newss;
  Serial.print(F("RSSI = ")); Serial.print(ss); Serial.print(": ");
  if (ss == 0) newss = -115;
  if (ss == 1) newss = -111;
  if (ss == 31) newss = -52;
  if ((ss >= 2) && (ss <= 30)) {
    newss = map(ss, 2, 30, -110, -54);
  }
  Serial.print(newss); Serial.println(F(" dBm"));
}

//--------------------------------LOOP--------------------------------
void loop() {
  while (!( millisrolloverfix(millis() - lastLoopMillis) > loopPeriod)); //this is the loop delay except it goes consistently every loopPeriod seconds.
  lastLoopMillis += loopPeriod;
  //start temperature measure
    float temperature, humidity;
    digitalWrite(THERMISTORPOWER, HIGH);  //
    delay(CHANGESTATETIME);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(LEDBLINKTIME);
    measureTemp(&temperature, &humidity, &thermTempCurrent);
    delay(CHANGESTATETIME);
    digitalWrite(THERMISTORPOWER, LOW);  //
    digitalWrite(LED_BUILTIN, LOW);
  //end temperature measure
  measureVolts(&voltCurrent);
  //start check bilge
    digitalWrite(BILGEPOWER, HIGH);
    delay(CHANGESTATETIME);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(LEDBLINKTIME);
    checkBilge(&bilgeCurrent);
    //Serial.println("BILGE:");
    //Serial.println(digitalRead(BILGEPIN));
    //Serial.println(bilgeCurrent);
    delay(CHANGESTATETIME);
    digitalWrite(BILGEPOWER, LOW);
    digitalWrite(LED_BUILTIN, LOW);
  //end check bilge
  
  if (voltCurrent < voltMin) voltMin = voltCurrent;
  if (voltCurrent > voltMax) voltMax = voltCurrent;
  if (bilgeCurrent == 1) bilgeMax = 1; //different type of max pusher because boolean
  if (thermTempCurrent < thermTempMin) thermTempMin = thermTempCurrent; //
  if (thermTempCurrent > thermTempMax) thermTempMax = thermTempCurrent; //
  
  float latitude, longitude, speed_kph, heading, altitude, second;
  getLocation(&latitude, &longitude, &speed_kph, &heading, &altitude);
  bool readyToText = isReadyToText();
  if (readyToText && doSubscribe){
    textInformation(thermTempMin, thermTempMax, thermTempCurrent, voltMin, voltMax, voltCurrent, bilgeMax, bilgeCurrent, latitude, longitude, 0);
  }
  if (readyToText){
    resetMinMax();  
  }
  if (isReadyToDweet()){
    dweetInformation(thermTempMin, thermTempMax, thermTempCurrent, voltMin, voltMax, voltCurrent, bilgeMax, bilgeCurrent, latitude, longitude);
  }
  if ( ((thermTempCurrent * 1.8 + 32) < tempAlarmMin or voltCurrent < voltAlarmMin or bilgeCurrent == 1) and (isAlarmReady()) ){
    textInformation(thermTempMin, thermTempMax, thermTempCurrent, voltMin, voltMax, voltCurrent, bilgeMax, bilgeCurrent, latitude, longitude, 1);
    /*Serial.println(thermTempCurrent * 1.8 + 32);
    Serial.println(tempAlarmMin);
    Serial.println("DEBUG");*/
  }
    
  char toRead[INCOMING_MSG_LIMIT];
  int output = readLastText(toRead);
  if (output == 2){
    Serial.println("READING MESSAGE");
    Serial.println(toRead);
    Serial.println("DONE READING MESSAGE");
  }
  if (output == 1){
    Serial.println("Error in deleting SMS's. Odd.");
  }
  if (output == 0){
    Serial.println("No new messages to read");
  }
  bool flag = 1; //for seeing if to throw unknown command message
  if(toRead[0] == '!' and hasPermissions()){
    if (toRead[1] == 'S' and toRead[2] == 'C' and toRead[3] == 'H' and toRead[4] == 'E' and toRead[5] == 'D' and toRead[6] == 'U'\
    and toRead[7] == 'L' and toRead[8] == 'E'){
      //text(lastTexter, "Starting new text message schedule!");
      lastTextMillis = millis();
      flag = 0;
    }
    
    if (stringslice_eq(toRead, "GET", 1, 4)){
      textInformation(thermTempMin, thermTempMax, thermTempCurrent, voltMin, voltMax, voltCurrent, bilgeMax, bilgeCurrent, latitude, longitude, 0);
      dweetInformation(thermTempMin, thermTempMax, thermTempCurrent, voltMin, voltMax, voltCurrent, bilgeMax, bilgeCurrent, latitude, longitude);
      flag = 0;
    }

    if (stringslice_eq(toRead, "DWEET", 1, 6)){
      dweetInformation(thermTempMin, thermTempMax, thermTempCurrent, voltMin, voltMax, voltCurrent, bilgeMax, bilgeCurrent, latitude, longitude);
      flag = 0;
    }

    if (stringslice_eq(toRead, "DEBUG", 1, 6)){
      char messageChar[160];
      sprintf(messageChar, "millis():%lu\nlastTextMillis:%lu\nlastLoopMillis:%lu\nlastDweetMillis:%lu\nlastAlarmMillis:%lu", millis(), lastTextMillis, lastLoopMillis, lastDweetMillis, lastAlarmMillis);
      text(lastTexter, messageChar);
      flag = 0;
    }
    
    if (stringslice_eq(toRead, "UNSUBSCRIBE", 1, 12)){
      //text(lastTexter, "Unsubscribing!");
      doSubscribe = false;
      flag = 0;
    }
    
    if (stringslice_eq(toRead, "SUBSCRIBE", 1, 10)){
      //text(lastTexter, "Subscribing");
      doSubscribe = true;
      flag = 0;
    }
    if (stringslice_eq(toRead, "CLEARMINMAX", 1, 12)){
      //text(lastTexter, "Reset mins and maxes since last textIncrement");
      resetMinMax();
      flag = 0;
    }

    if (stringslice_eq(toRead, "ALARMTEMP", 1, 10)){
      tempAlarmMin = (toRead[11]-'0')*10 + (toRead[12]-'0');
      //text(lastTexter, "tempAlarmMin set!");
      Serial.println("Temp Alarm Min:");
      Serial.println(tempAlarmMin);
      flag = 0;
    }

    if (stringslice_eq(toRead, "ALARMVOLT", 1, 10)){
      voltAlarmMin = (toRead[11]-'0')*10 + (toRead[12]-'0') + (toRead[14]-'0')*0.1;
      //text(lastTexter, "voltAlarmMin set!");
      Serial.println("Volt Alarm Min:");
      Serial.println(voltAlarmMin);
      flag = 0;
    }

    if (stringslice_eq(toRead, "DELAYALARM", 1, 11)){
      unsigned long minuteDelay = (toRead[12]-'0')*100 + (toRead[13]-'0')*10 + (toRead[14]-'0');
      alarmTimerType = 1;
      lastAlarmMillis = millis();
      userAlarmDelay = minuteDelay*60*1000;
      //text(lastTexter, "tempAlarmMin set!");
      Serial.println("Delaying Alarm...");
      Serial.println(minuteDelay);
      flag = 0;
    }
    
    if (stringslice_eq(toRead, "SETSPEED", 1, 9)){
      int userinput = (toRead[17]-'0')*10 + (toRead[18]-'0');
      long multiplier = 0;
      if (toRead[15] == 'S'){multiplier = 1000;}
      if (toRead[15] == 'M'){multiplier = 60000;}
      if (toRead[15] == 'H'){multiplier = 3600000;}
      if (toRead[10] == 'T' and toRead[11] == 'E' and toRead[12] == 'X' and toRead[13] == 'T'){
        textIncrement = userinput*multiplier;
        Serial.println("Text Increment has been changed to");
        Serial.println(textIncrement);
      }
      if (toRead[10] == 'P' and toRead[11] == 'O' and toRead[12] == 'S' and toRead[13] == 'T'){
        dweetIncrement = userinput*multiplier;
        Serial.println("Dweet Increment has been changed to");
        Serial.println(dweetIncrement);  
      }
      if (toRead[10] == 'L' and toRead[11] == 'O' and toRead[12] == 'O' and toRead[13] == 'P'){
        loopPeriod = userinput*multiplier;
        lastLoopMillis = millis();
        Serial.println("Loop Period has been changed to");
        Serial.println(loopPeriod);
      }
      if (toRead[10] == 'R' and toRead[11] == 'I' and toRead[12] == 'N' and toRead[13] == 'G'){
        alarmRepeat = userinput*multiplier;
        Serial.println("Default Alarm Repeat has been changed to");
        Serial.println(alarmRepeat);
      }
      //text(lastTexter, "Acknowledged change to speed");
      flag = 0;
    }
    
    if (stringslice_eq(toRead, "HELP", 1, 5)){
      text(lastTexter, "Commands:\nIgnore ()'s, <> are arguments.\n!SCHEDULE\n!UNSUBSCRIBE and !SUBSCRIBE\n!GET (Measure)\n!CLEARMINMAX\n!ALARMTEMP <XX>\n!ALARMVOLT <XX.X>\n...");
      text(lastTexter, "...\n!DELAYALARM <XXX>(mins)\n!SETSPEED <TEXT/POST/LOOP/RING> <S/M/H> <XX>\n!HELP\n!COMMANDS: Learn what each command does");
      flag = 0;
    }

    if (stringslice_eq(toRead, "COMMANDS", 1, 9)){
      text(lastTexter, "Commands:\n-!SCHEDULE -- Changes the daily text time to current.\n!UNSUBSCRIBE and !SUBSCRIBE -- Changes if the admin recieves daily messages.\n...");
      text(lastTexter, "\n!GET -- Reports readings.\n!CLEARMINMAX -- Clears min/maxes since last text.\n!ALARMTEMP <XX> and !ALARMVOLT <XX.X> -- Sets temp/volt alarm-tripping zone.\n...");
      text(lastTexter, "\n!DELAYALARM <XXX>(mins) -- Sets the next alarm-delay.\n!SETSPEED <TEXT/POST/LOOP/RING> <S/M/H> <XX> -- Sets the repeat speed of a function, with time unit.\n...");
      text(lastTexter, "\n!HELP -- Show a more concise list.\n!COMMANDS -- Show this list.");
    }
    /*
    if (toRead[1] == 'T' and toRead[2] == 'E' and toRead[3] == 'M' and toRead[4] == 'P' and toRead[5] == 'L' and toRead[6] == 'A'\
    and toRead[7] == 'T' and toRead[8] == 'E'){

    }
    */
    if (flag){
      text(lastTexter, "Unknown Command... Please type !HELP for more info. ");
    }
  }
}

//--------------------------------FUNCTIONS--------------------------------
void powerOn() {
  digitalWrite(FONA_PWRKEY, LOW);
  // See spec sheets for your particular module
#if defined(SIMCOM_2G)
  delay(1050);
#elif defined(SIMCOM_3G)
  delay(180); // For SIM5320
#elif defined(SIMCOM_7000)
  delay(100); // For SIM7000
#elif defined(SIMCOM_7500)
  delay(500); // For SIM7500
#endif

  digitalWrite(FONA_PWRKEY, HIGH);
}

void serialSendSMS(const char *number, const char *message) {
  Serial.print("MESSAGE:");
  Serial.println(number);
  Serial.println(message);
  Serial.println(F("END OF MESSAGE. YOU SAVED 19 CENTS"));
}

bool stringslice_eq(const char* str, const char* cmp, size_t strstart, size_t strend) {//unsafe function which checks if two strings are equal with a first string starting index
    //safety requirements:
    //str and cmp are both pointers to accessible memory
    //length of cmp memory is greater than or equal to strend - strstart
    //length of str memory is greater than or equal to strend
    for (size_t i = strstart; i < strend; i++) {
        if (*(str + (char)i) != *(cmp + (char)(i - strstart))) {
            return false;
        }
    }
    return true;
}

void textInformation(double msgThermTempn, double msgThermTempx, double msgThermTemp, float msgVoltn, float msgVoltx, float msgVolt, bool msgBilgex, bool msgBilge, float msgLat, float msgLong, bool isEmergency){
  //delay(5000);
  char messageChar[160];
  int latSign = ((2 * !(msgLat < 0)) - 1);
  int longSign = ((2 * !(msgLong < 0)) - 1);
  int latitudeInt = latSign * floor(latSign * msgLat);
  long latitudeDec = (msgLat - latitudeInt) * 1000000 * latSign; //Cancel lat sign because no sign in decimal points
  int latitudeDec2 = latitudeDec % 1000;
  int latitudeDec1 = (latitudeDec - latitudeDec2) / 1000;
  int longitudeInt = longSign * floor(longSign * msgLong);
  long longitudeDec = (msgLong - longitudeInt) * 1000000 * longSign;
  int longitudeDec2 = longitudeDec % 1000;
  int longitudeDec1 = (longitudeDec - longitudeDec2) / 1000;

  /*float tempF = msgTemp * 1.8 + 32;
  int tempFSign = ((2 * !(tempF < 0)) - 1);
  int tempFInt = tempFSign * floor(tempFSign * tempF);
  int tempFDec = (tempF - tempFInt) * 10 * tempFSign;*/

  double tempThermF = msgThermTemp * 1.8 + 32;
  int tempThermFSign = ((2 * !(tempThermF < 0)) - 1);
  int tempThermFInt = tempThermFSign * floor(tempThermFSign * tempThermF);
  int tempThermFDec = (tempThermF - tempThermFInt) * 10 * tempThermFSign;

  double tempThermFx = msgThermTempx * 1.8 + 32; // version 2 for MAX
  int tempThermFxSign = ((2 * !(tempThermFx < 0)) - 1); // version 2 for MAX
  int tempThermFxInt = tempThermFxSign * floor(tempThermFxSign * tempThermFx); // version 2 for MAX
  int tempThermFxDec = (tempThermFx - tempThermFxInt) * 10 * tempThermFxSign; // version 2 for MAX

  double tempThermFn = msgThermTempn * 1.8 + 32; // version for MIN
  int tempThermFnSign = ((2 * !(tempThermFn < 0)) - 1); // version for MIN
  int tempThermFnInt = tempThermFnSign * floor(tempThermFnSign * tempThermFn); // version 2 for MIN
  int tempThermFnDec = (tempThermFn - tempThermFnInt) * 10 * tempThermFnSign; //  version 2 for MIN

  int voltnSign = ((2 * !(msgVoltn < 0)) - 1);
  int voltnInt = voltnSign * floor(voltnSign * msgVoltn);
  int voltnDec = (msgVoltn - voltnInt) * 10 * voltnSign;
  
  int voltxSign = ((2 * !(msgVoltx < 0)) - 1);
  int voltxInt = voltxSign * floor(voltxSign * msgVoltx);
  int voltxDec = (msgVoltx - voltxInt) * 10 * voltxSign;

  int voltSign = ((2 * !(msgVolt < 0)) - 1);
  int voltInt = voltSign * floor(voltSign * msgVolt);
  int voltDec = (msgVolt - voltInt) * 10 * voltSign;
  /*int humidInt = (int)msgHumid;*/
  if (isEmergency){
    sprintf(messageChar, "ALARM!\nMin Temp:%i.%i Current Temp:%i.%i Max Temp:%i.%i Min Volt:%i.%i Current Volt:%i.%i Max Volt:%i.%i Bilge:%i BilgeMax:%i Latitude:%i.%i%i Longitude:%i.%i%i",\
    tempThermFnInt, tempThermFnDec, tempThermFInt, tempThermFDec, tempThermFxInt, tempThermFxDec, voltnInt, voltnDec, voltInt, voltDec, voltxInt, voltxDec, msgBilge, msgBilgex, latitudeInt, \
    latitudeDec1, latitudeDec2, longitudeInt, longitudeDec1, longitudeDec2);
  } else {
    sprintf(messageChar, "Min Temp:%i.%i Current Temp:%i.%i Max Temp:%i.%i Min Volt:%i.%i Current Volt:%i.%i Max Volt:%i.%i Bilge:%i BilgeMax:%i Latitude:%i.%i%i Longitude:%i.%i%i",\
    tempThermFnInt, tempThermFnDec, tempThermFInt, tempThermFDec, tempThermFxInt, tempThermFxDec, voltnInt, voltnDec, voltInt, voltDec, voltxInt, voltxDec, msgBilge, msgBilgex, latitudeInt, \
    latitudeDec1, latitudeDec2, longitudeInt, longitudeDec1, longitudeDec2);
  }
  //sprintf(messageChar, "https://www.google.com/maps/?q=%i.%i%i,%i.%i%i", latitudeInt, latitudeDec1, latitudeDec2, longitudeInt, longitudeDec1, longitudeDec2);
  text(ADMIN_NUMBER, messageChar);
  /*Serial.println("THERMISTOR TEMPERATURE (TEMPORARY)");
  Serial.println(msgThermTemp * 1.8 + 32);*/
}

void text(const char *number, const char *message){  
  fona.sendSMS(number, message);
  serialSendSMS(number, message);
}

void dweetInformation(double msgThermTempn, double msgThermTempx, double msgThermTemp, float msgVoltn, float msgVoltx, float msgVolt, bool msgBilgex, bool msgBilge, float msgLat, float msgLong){ 
  char dweetURL[255];
  int latSign = ((2 * !(msgLat < 0)) - 1);
  int longSign = ((2 * !(msgLong < 0)) - 1);
  int latitudeInt = latSign * floor(latSign * msgLat);
  long latitudeDec = (msgLat - latitudeInt) * 1000000 * latSign; //Cancel lat sign because no sign in decimal points
  int latitudeDec2 = latitudeDec % 1000;
  int latitudeDec1 = (latitudeDec - latitudeDec2) / 1000;
  int longitudeInt = longSign * floor(longSign * msgLong);
  long longitudeDec = (msgLong - longitudeInt) * 1000000 * longSign;
  int longitudeDec2 = longitudeDec % 1000;
  int longitudeDec1 = (longitudeDec - longitudeDec2) / 1000;

  /*float tempF = msgTemp * 1.8 + 32;
  int tempFSign = ((2 * !(tempF < 0)) - 1);
  int tempFInt = tempFSign * floor(tempFSign * tempF);
  int tempFDec = (tempF - tempFInt) * 10 * tempFSign;*/

  double tempThermF = msgThermTemp * 1.8 + 32;
  int tempThermFSign = ((2 * !(tempThermF < 0)) - 1);
  int tempThermFInt = tempThermFSign * floor(tempThermFSign * tempThermF);
  int tempThermFDec = (tempThermF - tempThermFInt) * 10 * tempThermFSign;

  double tempThermFx = msgThermTempx * 1.8 + 32; // version for MAX
  int tempThermFxSign = ((2 * !(tempThermFx < 0)) - 1); // version for MAX
  int tempThermFxInt = tempThermFxSign * floor(tempThermFxSign * tempThermFx); // version for MAX
  int tempThermFxDec = (tempThermFx - tempThermFxInt) * 10 * tempThermFxSign; // version for MAX

  double tempThermFn = msgThermTempn * 1.8 + 32; // version for MIN
  int tempThermFnSign = ((2 * !(tempThermFn < 0)) - 1); // version for MIN
  int tempThermFnInt = tempThermFnSign * floor(tempThermFnSign * tempThermFn); // version for MIN
  int tempThermFnDec = (tempThermFn - tempThermFnInt) * 10 * tempThermFnSign; // version for MIN

  
  int voltnSign = ((2 * !(msgVoltn < 0)) - 1);
  int voltnInt = voltnSign * floor(voltnSign * msgVoltn);
  int voltnDec = (msgVoltn - voltnInt) * 10 * voltnSign;
  
  int voltxSign = ((2 * !(msgVoltx < 0)) - 1);
  int voltxInt = voltxSign * floor(voltxSign * msgVoltx);
  int voltxDec = (msgVoltx - voltxInt) * 10 * voltxSign;

  int voltSign = ((2 * !(msgVolt < 0)) - 1);
  int voltInt = voltSign * floor(voltSign * msgVolt);
  int voltDec = (msgVolt - voltInt) * 10 * voltSign;

  /*int humidInt = (int)msgHumid;*/
  sprintf(dweetURL, URL_STRING,\
  tempThermFnInt, tempThermFnDec, tempThermFInt, tempThermFDec, tempThermFxInt, tempThermFxDec, voltnInt, voltnDec, voltInt, voltDec, voltxInt, voltxDec, msgBilge, msgBilgex, latitudeInt, latitudeDec1,\
  latitudeDec2, longitudeInt, longitudeDec1, longitudeDec2);
  Serial.println("Just Sprinted... Dweeting");
  if(fona.postData("GET", dweetURL)){
    Serial.println("Dweet supposedly successful!");
  } else {
    Serial.println("Dweet supposedly failed...");
  }
}

void measureTemp(float *tempPass, float *humidPass, double *thermTempPass){
  float tempF;
  bool hasMeasured = 0;
  /*while (!hasMeasured) {
    if ((dht_sensor.measure(&*tempPass, &*humidPass))) {
      Serial.print("Temperature F:");
      tempF = *tempPass * 1.8 + 32;
      Serial.print(tempF);
      Serial.print("Humidity%:");
      Serial.print(*humidPass);
      hasMeasured = 1;
    }
  }*/
  int tempReading = analogRead(THERMISTORPIN);
  double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );       //  Temp Kelvin
  *thermTempPass = tempK - 273.15;            // Convert Kelvin to Celcius
  //NO DONT DO THIS THE CODE RELIES ON CELSIUS *thermTempPass = (*thermTempPass * 1.8) + 32; // Convert Celsius to Fahrenheit
}

void measureVolts(float *voltage){
  *voltage = analogRead(VOLTMEASUREPIN)*(5.0*((VOLTDIVKOHM+VOLTPULLDOWNKOHM)/VOLTPULLDOWNKOHM)/1023.0); //cancel the 1023 ADC, multiply by 5 (AREF), multiply by 4 (voltage divider circuit)
}

void checkBilge(bool *bilge){
  *bilge = digitalRead(BILGEPIN);
}

void getLocation(float *latPass, float *longPass, float *speedPass, float *headingPass, float *altPass){
  if (fona.getGPS(&*latPass, &*longPass, &*speedPass, &*headingPass, &*altPass)) {
    Serial.println(F("---------------------"));
    Serial.print(F("Latitude: ")); Serial.println(*latPass, 6);
    Serial.print(F("Longitude: ")); Serial.println(*longPass, 6);
    Serial.print(F("Speed: ")); Serial.println(*speedPass);
    Serial.print(F("Heading: ")); Serial.println(*headingPass);
    Serial.print(F("Altitude: ")); Serial.println(*altPass);
  } else {
    Serial.println("Failed to read GPS");
  }
}

bool isReadyToText(){
  if ( millisrolloverfix(millis()-lastTextMillis) > textIncrement){
    lastTextMillis = timestampoverflowfix(lastTextMillis + textIncrement);
    return true;
  } else {
    return false;
  }
}

bool isAlarmReady(){
  bool defaultrepeat = ( millisrolloverfix(millis()-lastAlarmMillis) > alarmRepeat);
  bool manualdelay = ( millisrolloverfix(millis()-lastAlarmMillis) > userAlarmDelay);
  /*Serial.println("Manual delay conditional value is:");
  Serial.println(manualdelay);
  Serial.println("Current millis value, lastAlarmMillis value, userAlarmDelay");
  Serial.println(millis());
  Serial.println(lastAlarmMillis);
  Serial.println(userAlarmDelay);*/ //debuggin the manual delay :(
  bool chosenConditionOutput = (alarmTimerType*manualdelay) + ((!alarmTimerType)*defaultrepeat);
  if (chosenConditionOutput){
    //Serial.println("Alarm is Ready!");
    alarmTimerType = 0;
    lastAlarmMillis = millis();
    return true;
  } else {
    //Serial.println("Alarm is not Ready!");
    return false;
  }
}

bool isReadyToDweet(){
  if ( millisrolloverfix(millis()-lastDweetMillis) > dweetIncrement){
    lastDweetMillis = timestampoverflowfix(lastDweetMillis + dweetIncrement);
    return true;
  } else {
    return false;
  }
}

void resetMinMax(){
  tempMax = tempCurrent;
  tempMin = tempCurrent;
  thermTempMax = thermTempCurrent;
  thermTempMin = thermTempCurrent;
  voltMax = voltCurrent;
  voltMin = voltCurrent;
  bilgeMax = bilgeCurrent;
}

int readLastText(char *incoming){
  int smsCount = fona.getNumSMS();
  unsigned int smsLen;
  if (smsCount > 0){
    fona.readSMS(0, incoming, INCOMING_MSG_LIMIT, &smsLen);
    fona.getSMSSender(0, lastTexter, 15);
    if(fona.deleteAllSMS()){
      return 2;
    } else {
      return 1;
    }
  } else {
    return 0;
  }
}

bool hasPermissions(){
  bool overallchecker = 0;
  bool checker = 1;
  for(int i = 0; i < 13; i++){ //in a normal phone number like "+15558675309", 13 is the index+1 of the last digit
    if (ADMIN_NUMBER[i] != lastTexter[i]){
        checker = 0;
    }
  }
  overallchecker+=checker;
  checker = 1;
  for(int i = 0; i < 13; i++){
    if (WHITELISTED_1[i] != lastTexter[i]){
        checker = 0;
    }
  }
  overallchecker+=checker;
  checker = 1;
  for(int i = 0; i < 13; i++){
    if (WHITELISTED_2[i] != lastTexter[i]){
        checker = 0;
    }
  }
  overallchecker+=checker;
  
  if (overallchecker > 0){
    return true;
  } else {
    Serial.println(lastTexter);
    Serial.println("May be trying to hack!!");
    return false;
  }
}
