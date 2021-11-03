#include <VL53L1X.h>                //Library for communication with the VL53L1X sensor. ("Continuous", https://github.com/pololu/vl53l1x-arduino)
#include "RTClib.h"                 //Library for controlling DS3231. (“DS3231_alarm”, https://github.com/adafruit/RTClib)

#include <LowPower.h>               //Standard libraries for Arduino sleep function.
#include <avr/sleep.h>

#include <IridiumSBD.h>             //Library for communucatoin between Arduino and Iridium modems.(https://github.com/mikalhart/IridiumSBD/blob/master/src/IridiumSBD.h)

//Declare Components
RTC_DS3231 rtc;
VL53L1X sensor;

//Declare Pins
int ROCK_BLOCK_RX_PIN = 1;              //Define the RX1 pin to RockBLOCK (data receiving pin.)
int ROCK_BLOCK_TX_PIN = 0;              //Define the TX0 pin to RockBLOCK (data transimitting pin).
int EXTERNAL_INTERRUPT_PIN = 3;         //Connected to 'SQW' pin of DS3231 RTC.
int RESET_PIN = 5;                      //Connected to Arduino Pro Mini 3.3 V onboard 'RST' pin.
int SNOW_DEPTH_SENSOR_PIN = 8;          //Connected to VL53L1X 'VIN'. 
int HEATING_ELEMENT_PIN = 7;            //Connected to the 270 ohm resitor.

//Declare the IridiumSBD object            //Not within the scope of this project.
//#define IridiumSerial Serial3            //Input the specific Iridium Serial.
//IridiumSBD modem(IridiumSerial);         
//IridiumSBD ssIridium;      
     
/*-----------------------------------------------------*/
//Manual Inputs
int distanceToZero = 1200;                  //Input the measured distance to the ice surface (deployment height from ice surface) in mm 
                                            //It was set to be 1200 mm as a demonstartion, the value needs to be changed on installation after manually measuring the distance.

int lightPenetrationDepth = 20;             //Input the light penetration depth in mm (assumed to be < 20 mm for Antarctic snow, see Chapter 3, Section 3.3).
int duration = 5000;                        //Input the desired measurement duration in ms considering power efficiency.

const int windowSize = 20;                 //Input the number of readings taken before finding the average. (higher - more accuracy with less noise, but slower in speed)
                                           //windowSize = 20 was chosen through trial and error, with consideration of time duration.

int16_t days = 0;                           //Inter-measurement periods, eg 6 hours 30 minutes (days, hours, minutes, seconds) = (0, 6, 30, 0)
int8_t hours = 6;                           //As per design, this was set to measure the snow depth every 6 hours ((0, 6, 0, 0); 4 times per day) for power efficient design.
int8_t minutes = 0;                         //The values can be changed if needed, as suffcient battery capacity was chosen.
int8_t seconds = 0;

/*-----------------------------------------------------*/
//Variable for Heating Element 
int temperature = 0;         

/*-----------------------------------------------------*/
//Variables for Moving Average Filter
int index = 0;
int window[windowSize];                     //Distance readings from the VL53L1X sensor is stored in the "window" array. (see Chapter 5, Section 5.3)
int movingTotalDistance = 0;                //Running total of distance readings
int movingAverageDistance = 0;              //Moving average distance of the number of readings
int snowDepthFiltered = 0;                  //Resulting Snow Depth Measurement
 
/*-----------------------------------------------------*/
//Variable for internal counter millis()
unsigned long counter = 0;

/*-----------------------------------------------------*/
//Variables for RTC (Time vs Snow Depth)
String currentYY_BINARY;
String currentMM_BINARY;
String currentDD_BINARY;
String currentHOUR_BINARY;
String currentMIN_BINARY;
String currentSEC_BINARY;

/*-----------------------------------------------------*/
//Variables for Data Transmission via UART to RockBLOCK 9603
String snowDepthFiltered_BINARY;
String currentTime_BINARY;
String myURL;                                  //The binary string containing snow depth vs time data. This data is to be sent to the Iridium moden via UART.

/*-----------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() {
/*-----------------------------------------------------*/
  //Set Pin Modes and Initial States
  //Reset Pin (after waking up, Arduino is reset (same effect as pressing the physical reset button))
  
  digitalWrite (RESET_PIN, HIGH);         //First line of code that is ran once the board is reset. Terminates reset to prevent the reset happening repetitively.
  pinMode(RESET_PIN,OUTPUT);              //Reset pin needs to be brought 'LOW' in order to be triggered.
  
  //Snow Depth Sensor Pin
  pinMode(SNOW_DEPTH_SENSOR_PIN, OUTPUT);   
  digitalWrite (SNOW_DEPTH_SENSOR_PIN, LOW);  //VL53L1X initially power-down to ensure that the sensor is not operating below its minimum operating temperature of -20 ℃. 
  
  //Heating Element Pin
  pinMode(HEATING_ELEMENT_PIN, OUTPUT);
  digitalWrite (HEATING_ELEMENT_PIN, LOW);   //The ehating element is only initiated when temperature < -15 ℃ to prevent any wastage of power. 
 
/*-----------------------------------------------------*/
  //Initialise RTC
  rtc.begin();
    
  if(rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // this will adjust to the date and time at compilation
  }
   
  rtc.disable32K();            //32K Pin on RTC unused, therefore disabled

  rtc.clearAlarm(1);         //clear any existing alarm (there are 2 alarms on RTC)
  rtc.clearAlarm(2);
    
  rtc.writeSqwPinMode(DS3231_OFF);  // Terminate oscillating signals at SQW Pin, otherwise setAlarm1 will fail
    
  rtc.disableAlarm(1);               //disable alarms after waking up to prevent the alarms being overlooked
  rtc.disableAlarm(2);

/*-----------------------------------------------------*/
  //Initialise I2C
  Serial.begin(9600);            //Serial communication baud rate is set to be 9600 bps for communication between sensors and microcontroller (can be changed if necessary).
  Wire.begin();                  //Initiates the standart 'Wire' library and join the I2C bus as a master or slave (initiates I2C communication).

/*-----------------------------------------------------*/
}
//End of void setup()

//Start of void loop()
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void loop() {
  
  temperatureMeasure();
  
  if (temperature < -15){             //Check if temperature < -15 ℃. If "TRUE", initiate the heating element.
                                      //If "FALSE", skip to snowDepthMeasurementInit().
    heatingElementInit();
  }
  
  snowDepthMeasurementInit();
  
  
  getTime();
  
  
  //dataTransmission();
  
  
  setNextAlarm();
  
  
  goToSleep();
  

  while(true) {};           // Infinite loop of empty statement executed to terminate/trap the 'void loop()'
  
}
//End of void loop()

/*-----------------------------------------------------*/
void temperatureMeasure(){

  int analogVoltage = analogRead(1);  //From anolog pin 1(pin A1, to which the Vout of TMP36 is connected) analogRead() the voltage reading from the temperature sensor.
                                      //The resolution is in 10mV/degree centigrade with 500 mV offset to allow for negative temperatures.

  //Analog to Digital Conversion
  float voltage = analogVoltage * (3.3/1024);       //Converting that analog reading to voltage in digital form, for 3.3v Arduino Pro Mini 3.3V use 3.3 as a multiplier.
                                                    //The number 0-1023 from the ADC is then converted into 0-3300mV (= 3.3V)by the 1/1024 multiplier.

  int temperature = (voltage - 0.5) * 100 ;    //Converting from 10 mv per degree with 500 mV offset to celsius degrees ((voltage - 500mV) times 100).
                                       
  delay(100);
}


/*-----------------------------------------------------*/
void heatingElementInit(){
  
  while (temperature < -15){         //Power on the heating element (270 ohm resistor) while temperature < -15 ℃.
    
    digitalWrite (7, HIGH);
    delay (5000);                    //Once the heating element is initiated, wait 5 seconds for the temperautre to increase.
    
    temperatureMeasure();            //Measure and update the current temperature, if temperature < -15 ℃, reapeat the while loop.
                                     
    delay(50);
  }
  
  digitalWrite(7, LOW);             //If temperautre > 15 ℃, power down the heating element.
  digitalWrite(8, HIGH);            //Power on the VL53L1X sensor and exit to the next step, snowDepthMeasurementInit().
  
  delay(50);
}
/*-----------------------------------------------------*/
void snowDepthMeasurementInit(){
  
                     
  
  if (digitalRead(SNOW_DEPTH_SENSOR_PIN == HIGH)){
    
    sensorInit();                  //Initiate and setup the sensor, see void sensorInit().
    
    //Start Internal Counter
    int counter = millis();                         //Start internal counter.
  
    //Start measuring and filtering for the set duration
    while (counter <= duration){
    
      //Compentsation for Light Penetration Effect
      int nextDistance = sensor.read();                           //Read the next sensor value (distance to snow surface in mm).
      nextDistance = nextDistance - lightPenetrationDepth;        //Compensation for the effect of light penetration.

      //Moving Average Filter
      movingTotalDistance = movingTotalDistance - window[index];    //Remove the oldest entry in window from the movingTotal.
      window[index] = nextDistance;                                  //Add the most recent reading to the window.
      movingTotalDistance = movingTotalDistance + nextDistance;     //Add the most recent reading to the movingTotal.
    
      index = (index + 1) % windowSize;                             //Increment the index. If the indext exceeds the window size, reset to index = 0.
    
      movingAverageDistance = movingTotalDistance / windowSize;     //Divide the movingTotal by the windowSize for the movingAverage result.

      delay(50);                                                    //Delay added to ensure accurate measurements and prevent possible errors.
    }
  
  snowDepthFiltered = distanceToZero - movingAverageDistance;   //The latest filtered value is used to find the snow depth.

  snowDepthFiltered_BINARY = (snowDepthFiltered, BIN);          //Converts the measured and filtered snow depth from decimal to binary 
                                                                //for UART communication with the Iridium RockBLOCK 9603.
  
  digitalWrite(SNOW_DEPTH_SENSOR_PIN, LOW);   //Power down VL53L1X sensor as snow depth measurement has been completed.
 
  delay(100);                            
  
  //Move on to the next step, getTime().
  }
}

/*-----------------------------------------------------*/
void sensorInit(){
  //Initialise the VL53L1X Senosr
  sensor.init();
  sensor.setDistanceMode(VL53L1X::Short);         //Measruement mode set to Short Mode for highest accuracy and least arrected by ambient light, 
                                                  //however, range limited to 136 cm (it could still measure upto 150 cm during testing).

  sensor.setROISize(4, 4);                       //Minmum Region of Interest (ROI), 4x4, for minimum Field of View (FOV) of 15 degrees. (Max. ROI (16x16) for max. FOV of 27 degrees).
    
  sensor.setMeasurementTimingBudget(45000);     //Time budget allocated to be 45000 μs (45 ms), higher timing budget for higher accuracy, but takes longer to measure.
  sensor.startContinuous(50);                   //Start continuous readings at a rate of one measurement every 50 ms (the inter-measurement period). 
                                                //This period should be at least as long as the timing budget.
                                                //Inter-measurement period is recommended to be set at least 4ms higher than the timing budget
  delay(100);
}

/*-----------------------------------------------------*/
void getTime(){
  
  DateTime now = rtc.now();           //Receives the time data from the DS3231 RTC.
  
  //Get Current Time in Integer
  int currentYY = now.year();
  int currentMM = now.month();
  int currentDD = now.day();
  int currentHOUR = now.hour();
  int currentMIN = now.minute();
  int currentSEC = now.second();

  //Type Conversion to Binary String
  currentYY_BINARY = (currentYY, BIN);
  currentMM_BINARY = (currentMM, BIN);
  currentDD_BINARY = (currentDD, BIN);
  currentHOUR_BINARY = (currentHOUR, BIN);
  currentMIN_BINARY = (currentMIN, BIN);
  currentSEC_BINARY = (currentSEC, BIN);

  currentTime_BINARY = (currentTime_BINARY =  currentYY_BINARY + "," + currentMM_BINARY +"," + currentDD_BINARY + "," 
                         + currentHOUR_BINARY + "," + currentMIN_BINARY + "," + currentSEC_BINARY);
  
  //eg 11111100101,1010,10000,10111,110111,111000 (YY,MM,DD,h,m,s)
  
  delay(100);
  
  //Move to dataTransmission() for UART communication with the RockBLOCK 9603.
}

/*-----------------------------------------------------*/
/*void dataTransmission(){
  //Start the Serial Port
  ssIridium.begin(19200);                   //Baud rate set to be the same as the default of RockBLOCK 9603 (19200 bps)
  ssIridium.listen();                       //Initiates the Iridium modem to start receving data
 
  myURL = "," + currentTime_BINARY + "," +  snowDepthFiltered_BINARY + ",0";           //eg ,11111100101,1010,10000,10111,110111,111000,10000010,0 (,YY,MM,DD,h,m,s,snowDepth,0)
                                                                                       //"," to indicate the start of URL, and "0" required as a terminator byte for RockBLOCK servers.
                                                                                       //The transmitted data will contain date, time and snow depth in binary numbers, separated by ",".

  adaptiveRetry()           //Function which sends the data to the satellite server and waits until successful, attempts retries if needed.
  
  ssIridium.end();          //Data successfully sent, the function ends.

  delay(100);               
  //Since snow depth measurement process has been completed, prepare to enter sleep, setNextAlarm().  
}
*/

/*-----------------------------------------------------*/
void setNextAlarm(){
  //rtc.setAlarm1(rtc.now() + TimeSpan(20),DS3231_A1_Second);     //This mode triggers the alarm when the seconds match. 20 seconds for this case.
  
  rtc.setAlarm2(rtc.now() + TimeSpan(days, hours, minutes, seconds), DS3231_A2_Minute);     //Alarm is set as previously defined (after 6 hours). 
  
  delay(100);

  //Now that the alarm is set and the Arduino is ready to sleep, goToSleep().
}

/*-----------------------------------------------------*/
void goToSleep(){
  
  //Define the Interrupt (External Interrupt)
  attachInterrupt(digitalPinToInterrupt(EXTERNAL_INTERRUPT_PIN), onAlarm, LOW);    //Wake the Arduino from sleep when the "RTC_INTERRUPT_PIN" pin is triggered LOW 
                                                                                    //(falling due to active-low signal). When awake, call the "onAlarm" function.
  
  //Power down Arduino (Sleep)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);            //Set sleep mode as power-down mode (completely asleep with the Arduino CPU down).
  
  sleep_enable();
  
  sleep_mode();
  
  sleep_cpu();
  
  
  //First thing to do is disable sleep
  sleep_disable();                         //Prevents the Arduino from going back to sleep once awake.

  delay(100);  

  //The Arduino is asleep until the next external interrupt (RTC alarm) occurs.
  //When alarm is triggered (EXTERNAL_INTERRUPT_PIN), LOW), onAlarm() is called.
}

/*-----------------------------------------------------*/
void onAlarm(void){
  //This will bring the Arduino back from sleep
  //Resets the Arduino once awake to ensure that all functions are runs from beginning. 
  //This has the same effect as pressing on the onboard physical reset button.
  
  digitalWrite(RESET_PIN, LOW);      //Reset(RST) pin needs to be brought low for the borad reset to be triggered.                   
  
  delay(100);
 }

//End of code
//Back to void setup().

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
