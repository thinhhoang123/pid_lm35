#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <SimpleKalmanFilter.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
SimpleKalmanFilter antiJamming(2, 2, 0.1);

#define BT    A3
#define BT1   A1 
#define BT2   A2 
#define relay 8
// #define pwm   10
// #define in1   9
// #define in2   8

// Analog output pin
#define outputPin 9

// thermistor analog pin
#define THERMISTORPIN A0
// how many samples to take and average
#define NUMSAMPLES 5
// how long between pid/sampling
#define SAMPLETIME 1000
//Define Variables we'll be connecting to
double Setpoint, currentTemp, Output;
//Specify the links and initial tuning parameters
double kp = 15, ki =.3, kd = 0; //PID parameters
PID myPID(&currentTemp, &Output, &Setpoint, kp, ki, kd, DIRECT);
int lastTemp = 0;
int mode = 0;
int setupTemperature,gt_filter;
int TOLERANCE_TEMP = 5;

void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  pinMode(outputPin, OUTPUT);
  
  //initialize PID setpoint *C
  //Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(SAMPLETIME);
  myPID.SetTunings(kp, ki, kd);
  myPID.SetOutputLimits(0,255);
  //pid Autotuner

  // Pin input
  // pinMode (LM35, INPUT);
  pinMode (BT1,INPUT_PULLUP);
  pinMode (BT2,INPUT_PULLUP);
  pinMode (BT,INPUT);
  
  pinMode (relay, OUTPUT);
 
  // LCD init
  lcd.init();
  lcd.backlight();

}

void loop() {
  buttonChangeMode();
  setupMode();

  if(mode == 1) {
    write_lcd(6, 0, "OPERATION");
    write_lcd(0, 1, "TEMP:");
    getTemperature();
    buttonChangeMode();
  }
}
////////////////////////////////////////////////////////////////
void getTemperature(){
  uint8_t i;
  double average = 0;
  
  Setpoint = setupTemperature;
  
  buttonChangeMode();

  if (Serial.available() > 0) {
    // get incoming byte:
    Setpoint = Serial.parseFloat();
  }
  
  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    average += analogRead(THERMISTORPIN);
    delay(10);
  }

  average /= NUMSAMPLES;
  currentTemp=resistanceToC(inputToResistance(average));
  turnOnRelay(currentTemp, lastTemp);
  
  myPID.Compute();

  analogWrite(outputPin, Output);

  lcd.setCursor(5, 1);
  lcd.print(currentTemp);
  
  lastTemp = currentTemp;
  delay(SAMPLETIME);  
}
//////////////////////////////////////////////////////////////////////////////
void buttonChangeMode() {
  if(digitalRead(BT1) == 0) {
    delay(300);
    lcd.clear();
    mode++;
    if(mode > 1) mode = 0;
  }  
}
//////////////////////////////////////////////////////////////////////////////
void setupMode()
{
  buttonChangeMode();
  //lcd.clear();
  if(mode == 0) {

    write_lcd(0, 1, "TEMP:");
    write_lcd(6, 0, "SETUP");
    Serial.write('0');
    analogWrite(outputPin, 0);
    digitalWrite(relay, LOW); 
   
    gt_filter = antiJamming.updateEstimate(analogRead(BT));
    setupTemperature = map(gt_filter, 0, 1023, 0, 200);

    lcd.setCursor(5, 1);
    if((setupTemperature/100%10)!=0) lcd.print(setupTemperature/100%10);  
    else lcd.print(' '); 
    lcd.print(setupTemperature/10%10);
    lcd.print(setupTemperature%10);
  }
}
//////////////////////////////////////////////////////////////////////////////
void turnOnRelay(int temp, int lastTemp) {
  int minTemp = setupTemperature - TOLERANCE_TEMP;
  int maxTemp = setupTemperature + TOLERANCE_TEMP;
  if((temp > minTemp && temp < maxTemp) && (lastTemp > minTemp && lastTemp < maxTemp)) {
    digitalWrite(relay, HIGH);
    Serial.write('1');
  } 
  else {
    digitalWrite(relay, LOW);
    Serial.write('0');
  }

}
//////////////////////////////////////////////////////////////////////////////
void write_lcd(int x, int y, char content[])
{
  lcd.setCursor(x, y);
  lcd.print(content);
}
/*////////////////////////////////////////////////////////////////////////////
  Funtion to convert the input value to resistance  
  The value of the 'other' resistor
*/
double inputToResistance(double input) {
  double SERIESRESISTOR = 10000;
  input = 1023 / input - 1;
  return SERIESRESISTOR / input;
}
/*///////////////////////////////////////////////////////////////////////////
  Funtion to convert resistance to c
  Temp/resistance for nominal
*/
double resistanceToC(double resistance) {
  double THERMISTORNOMINAL = 118000;
  double TEMPERATURENOMINAL = 25;
  // beta coefficent
  double BCOEFFICIENT = 3950;
  double steinhart;
  steinhart = resistance / THERMISTORNOMINAL; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15; // convert to C
  return steinhart;
}