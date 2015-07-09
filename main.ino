  #include <Time.h>
  #include <Wire.h>
  #include <OneWire.h>
  #include <SPI.h>
  #include <Adafruit_GFX.h>
  #include <TFT_ILI9163C.h>
  #include "DHT.h"
  /*
    AnalogReadSerial
   Reads an analog input on pin 0, prints the result to the serial monitor.
   Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
   
   This example code is in the public domain.
   */

  // Amount of averaging data points
  const int AVERAGING = 25;
  int averageIdx;


  /*************DISPLAY*****************************************************/
  #define BLACK   0x0000
  #define BLUE    0x001F
  #define RED     0xF800
  #define GREEN   0x07E0
  #define CYAN    0x07FF
  #define MAGENTA 0xF81F
  #define YELLOW  0xFFE0  
  #define WHITE   0xFFFF

  // Display
  #define __CS 10
  #define __DC 9
  #define __RST 8

  TFT_ILI9163C tft = TFT_ILI9163C(__CS, __DC, __RST);
  int printing;
  /************* DISPLAY *****************************************************/


  /************* DHT SENSOR *****************************************************/
  #define DHTTYPE DHT11   // DHT 21  (AM2301)
  #define DHTPIN 2

  DHT dht(DHTPIN, DHTTYPE);

  /************* WATER TEMP. SENSOR *****************************************************/
  OneWire  ds(3);  // on pin 3 (a 4.7K resistor is necessary)

  /*************CONFIG******************************************************/
  const int PUMP_ON_VALUE = 0;        // '1' if output must be high to turn on the pump
  /*************CONFIG******************************************************/


  /*************PINS********************************************************/
  const int POWER_PIN = 7;             // the number of the power to the potmeters pin
  const int ON_TIMER = A0;            // the analog reading pin for on time
  const int REPEAT_TIMER = A1;        // the analog reading pin for the repeat time
  const int PUMP_POWER = A2;
  const int PUMP_PIN = 12;
  /*************PINS********************************************************/


  /*************TIMING******************************************************/
  int onAverager[AVERAGING];
  int repeatAverager[AVERAGING];

  long onTime;            // in milliseconds
  long prevOnTime;
  long repeatTime;        // in milliseconds
  long prevRepeatTime;
  long countingSeconds;
  /*************TIMING******************************************************/


  /*************PUMP********************************************************/
  int pumpOn;

  int pumpPowerAverager[AVERAGING];

  int pumpPower;
  int prevPumpPower;

  int pumpPwmTime;
  /*************PUMP********************************************************/

  float humidity;
  float temperature;
  float waterTemp;

  // the setup routine runs once when you press reset:
  void setup() {
    for(int i=0; i<AVERAGING; i++){
      onAverager[i] = 0;
      repeatAverager[i] = 0;
      pumpPowerAverager[i] = 0;
    }
    averageIdx = 0;

    pinMode(POWER_PIN, OUTPUT);
    pinMode(PUMP_PIN, OUTPUT);      
    digitalWrite(PUMP_PIN, !PUMP_ON_VALUE);

    printing = 0;
    tft.begin();  
    dht.begin();

    onTime = 0;
    prevOnTime = 1;

    repeatTime = 0;
    prevRepeatTime = 1;

    countingSeconds = 0;

    pumpOn = ! PUMP_ON_VALUE;

    pumpPower = 0;
    prevPumpPower = 1;

    pumpPwmTime = 0;
    // Timers
    // cli();
    // //set timer1 interrupt at 1Hz
    // TCCR1A = 0;// set entire TCCR1A register to 0
    // TCCR1B = 0;// same for TCCR1B
    // TCNT1  = 0;//initialize counter value to 0
    // // set compare match register for 1hz increments
    // OCR1A = 15624;// (15978 voor huidige bordje, 15264 originee) = (16*10^6) / (1*1024) - 1 (must be <65536)
    // // turn on CTC mode
    // TCCR1B |= (1 << WGM12);
    // // Set CS10 and CS12 bits for 1024 prescaler
    // TCCR1B |= (1 << CS12) | (1 << CS10);  
    // // enable timer compare interrupt
    // TIMSK1 |= (1 << OCIE1A);

    // TCCR0A = 0;// set entire TCCR2A register to 0
    // TCCR0B = 0;// same for TCCR2B
    // TCNT0  = 0;//initialize counter value to 0
    // // set compare match register for 1khz increments
    // OCR0A = 16;// = (16*10^6) / (248000*64) - 1 (must be <256)
    // // turn on CTC mode
    // TCCR0A |= (1 << WGM01);
    // // Set CS01 and CS00 bits for 64 prescaler
    // TCCR0B |= (1 << CS01) | (1 << CS00);   
    // // enable timer compare interrupt
    // TIMSK0 |= (1 << OCIE0A);
    // sei();

    tft.fillScreen(YELLOW);
    tft.fillScreen();
    tft.setCursor(0, 5);
    tft.print("Pump pwr: 100 %");
    tft.setCursor(0, 20);
    tft.print("On time : 0:00:00");
    tft.setCursor(0, 35);
    tft.print("Interval: 0:00:00");
    tft.setCursor(0, 50);
    tft.print("Humidity: 00.00 %");
    tft.setCursor(0, 65);
    tft.print("Air Temp: 00.00 C");
    tft.setCursor(0, 80);
    tft.print("O2  Temp: 00.00 C");
    // Serial.begin(9600);
    // Serial.println("Pump setup complete");
  }

  ISR(TIMER0_COMPA_vect){//timer0 interrupt
    if (pumpPwmTime < 100)
      pumpPwmTime++;
    else
      pumpPwmTime = 0;

    if (pumpPwmTime < pumpPower) {
      digitalWrite(PUMP_PIN, pumpOn);
    }
    else
      digitalWrite(PUMP_PIN, !PUMP_ON_VALUE);
  }

  ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz
    if (countingSeconds < (repeatTime / 1000) - 1)
      countingSeconds++;
    else
      countingSeconds = 0;

    if (countingSeconds < (onTime / 1000)) {
      pumpOn = PUMP_ON_VALUE;
    }
    else {
      pumpOn = !PUMP_ON_VALUE;
    }
  }

  void printTime(long secs) {
    int seconds = (int)secs % 60;
    int minutes = (int)(secs / 60) % 60;
    int hours = (int)(secs / 3600);

    if (hours > 10) {
      tft.print("X");
    }
    else {
      tft.print(hours);
    }
    tft.print(":");

    if (minutes < 10) {
      tft.print("0");
    }
    tft.print(minutes);

    tft.print(":");

    if (seconds < 10) {
      tft.print("0");
    }
    tft.print(seconds);
  }

  void printPower(int power) {
    if (power < 10) {
      tft.print(" ");
    }
    else if (power < 100) {
      tft.print(" ");
    }
    tft.print(power);
  }

  void printHumidity(float h) {
    tft.print(h);
  }

  void printTemperature(float t) {
    tft.print(t);
  }

  void printWaterTemp(float wt) {
    tft.print(wt);
  }

  float getWaterTemp() {
    byte data[12];
    byte addr[8];
   
    if ( !ds.search(addr)) {
      ds.reset_search();
      return -1000;
    }

    if (OneWire::crc8( addr, 7) !=addr[7]) {
      tft.print("CRC not valid");
      return -1000;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44,1);

    byte present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);

    for(int i = 0; i < 9; i++) {
        data[i] = ds.read();
    }

    ds.reset_search();

    return ((data[1] << 8) | data[0]) / 16;
  }

  void printDisplay() {
    if (printing >= 100) {
      if (pumpPower != prevPumpPower) {
        tft.fillRect(60, 5, 18, 10, BLACK);
        tft.setCursor(60, 5);
        printPower(pumpPower);
        prevPumpPower = pumpPower;
      }
      
      if (onTime != prevOnTime) {
        tft.fillRect(60, 20, 42, 10, BLACK);
        tft.setCursor(60, 20);
        printTime(onTime / 1000);
        prevOnTime = onTime;
      }

      if (repeatTime != prevRepeatTime) {
        tft.fillRect(60, 35, 42, 10, BLACK);
        tft.setCursor(60, 35);
        printTime(repeatTime / 1000);
        prevRepeatTime = repeatTime;
      }

      tft.fillRect(60, 50, 27, 10, BLACK);
      tft.setCursor(60, 50);
      printHumidity(humidity);

      tft.fillRect(60, 65, 27, 10, BLACK);
      tft.setCursor(60, 65);
      printTemperature(temperature);

      tft.fillRect(60, 80, 27, 10, BLACK);
      tft.setCursor(60, 80);
      printWaterTemp(waterTemp);

      printing = 0;
    }
    printing++;
  }

  void readPins() {
    digitalWrite(POWER_PIN, HIGH);
    // read the input on analog pin 0:
    onAverager[averageIdx] = analogRead(ON_TIMER);
    repeatAverager[averageIdx] = analogRead(REPEAT_TIMER);
    pumpPowerAverager[averageIdx] = analogRead(PUMP_POWER);
    digitalWrite(POWER_PIN, LOW);
    averageIdx++;
    if (averageIdx >= AVERAGING)
      averageIdx = 0;

    long onValue = 0;
    long repeatValue = 0;
    long pumpPowerValue = 0; 
    for(int i=0; i<AVERAGING; i++){
      onValue += onAverager[i];
      repeatValue += repeatAverager[i];
      pumpPowerValue += pumpPowerAverager[i];
    }

    onTime = (long)(1000 * onValue / double(AVERAGING));
    repeatTime = (long)((1000 * repeatValue / double(AVERAGING)) * 7 + 600000);

    pumpPower = 100 * ((pumpPowerValue / AVERAGING) + 1) / 1024;
    
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    waterTemp = getWaterTemp();
  }

  // the loop routine runs over and over again forever:
  void loop() {

    readPins();
    printDisplay();

  }
