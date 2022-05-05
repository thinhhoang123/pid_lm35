#include <PID_v2.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

#define BT    A0
#define BT1   A1 
#define BT2   A2 
#define LM35  A3
#define relay 11
#define pwm   10
#define in1   9
#define in2   8

unsigned long PID_TIME = 3000;

double set_point ; // will be the desired value
double input_lm35; // Sensor lm35
double output ; //LED
double kp = 3.6, ki = 0, kd = 206 ; //PID parameters
PID myPID(&input_lm35, &output, &set_point, kp, ki, kd, DIRECT); //create PID instance
 
int mode = 0;
int setup_temperature;
unsigned long timer_1 = 0;

void setup() {
  Serial.begin(9600);  

  // Pin input
  // pinMode (LM35, INPUT);
  pinMode (BT1,INPUT_PULLUP);
  pinMode (BT2,INPUT_PULLUP);
  pinMode (BT,INPUT);

  // Pin output
  pinMode (relay, OUTPUT);
  pinMode (pwm,  OUTPUT);
  pinMode (in1, OUTPUT);
  pinMode (in2,  OUTPUT);
  
  // LCD init
  lcd.init();
  lcd.backlight();

  // PID init
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(kp, ki, kd);
  myPID.SetSampleTime(PID_TIME);
  myPID.SetOutputLimits(0,255);


  // Set chiều quay cho quạt
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

}

void loop() {
  button();
  setup_mode();
  if(mode == 1)
  {
    control_temperature();
    write_lcd(4, 0, "OPERATION");
    write_lcd(0, 1, "TEMP:");
    button();
  }
}
//================================================
void button(){
  if(digitalRead(BT1) == 0) 
  {
    delay(500);
    lcd.clear();
    mode++;
    if(mode > 1) mode = 0;
  }
}
//================================================
void setup_mode()
{
  if(mode == 0)
  {
    write_lcd(0, 1, "TEMP:");
    write_lcd(6, 0, "SETUP");

    analogWrite(pwm, 0);

    setup_temperature = map(analogRead(BT), 0, 1023, 0, 100);

    lcd.setCursor(5, 1);
    lcd.print(setup_temperature/10%10);
    lcd.print(setup_temperature%10);
  }
}
//========================================
void control_temperature()
{
  set_point = setup_temperature;
  float read_temp = read_lm35();
  myPID.Compute();

  if((millis() - timer_1) > PID_TIME)
  {
    if(read_temp >= setup_temperature)
    {  
      analogWrite(pwm,100);
      write_lcd(11, 1, "FAN:MIN");
     
    } 
    else
    {
      analogWrite(pwm,255);
      write_lcd(11, 1, "FAN:MAX");
  
    }
    
    int xung = constrain((int)output,0,255);
    analogWrite(relay,xung); 

    timer_1 = millis();
  }
}

//========================================
void write_lcd(int x, int y, char content[])
{
  lcd.setCursor(x, y);
  lcd.print(content);
}
//========================================
float read_lm35(){
  input_lm35 = analogRead(LM35);
  float voltage = input_lm35 * 5.0 / 1024.0;
  float temp = voltage * 100.0;
  delay(1000);
  lcd.setCursor(5, 1);
  lcd.print(temp);
  Serial.print(temp);
  return temp;
}

