#include <Arduino.h>
#include <SimpleFOC.h>
#include <Adafruit_GFX.h> // Core graphics library
// #include <Adafruit_ST7735.h> // Hardware-specific library for ST7789
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
// #include <SdFat.h>
#include <Adafruit_SPIFlash.h>
// #include "Adafruit_TSC2007.h"

#include <STM32SD.h>

// If SD card slot has no detect pin then define it as SD_DETECT_NONE
// to ignore it. One other option is to call 'card.init()' without parameter.
// #ifndef SD_DETECT_PIN
#define SD_DETECT_PIN SD4
// #endif

Sd2Card card;
SdFatFs fatFs;

#define SD_DETECT_PIN PD4
// #define M_PWM PA10
#define MEM_CS PD6
#define TFT_CS PB12
#define TFT_RST PD9 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC PA5
#define TFT_MOSI PD7
#define TFT_MISO PB4
#define TFT_SCLK PB3
#define I2C_SDA PB9
#define I2C_SCL PB8
#define touch_interrupt_pin PB14
#define sensor1_adc PC4
#define sensor2_adc PB1

TwoWire I2C_main(PB9, PB8);
SPIClass SPI_1(TFT_MOSI, TFT_MISO, TFT_SCLK);
Adafruit_FlashTransport_SPI flashTransport(MEM_CS, SPI_1);
Adafruit_SPIFlash flash(&flashTransport);
// Adafruit_TSC2007 touch;

// SPIClass SPI_3(PB15, PB14, PB13);
//  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
// String request;
const float oneRev = 1700;
float eGear = 1;
HardwareSerial serial_hw(PB11, PB10);
// MagneticSensorSPI sensorMA730 = MagneticSensorSPI(MA730_SPI, SPI_CS_PIN);
// MagAlpha magAlpha;
double encoderAngle;
double initialAngle;
float initialPos;
float hallAngle;
float counter;
float window = 0.03;
bool tester;
const int measurePin = 2;
const int pwmA = PA1; // works  also PA10
const int pwmB = PA2; // works
const int pwmC = PA3; // works
// const int pwmA = PB5; // works  also PA10
// const int pwmB = PB1; // works
// const int pwmC = PB0; // works
const int enable = PA4;
const int encApin = PE4; // pc0
const int encBpin = PE5; // pc1
const int encCpin = PE6; // pb1
float position0 = 0;
float position5 = oneRev / 72;
float position90 = oneRev / 4;
float position180 = oneRev / 2;
float positionP1 = 0;
float positionP2 = 1700;
u_int32_t timestamp = 0;
u_int32_t timestamp1 = 0;
u_int32_t timestamp2 = 0;
u_int32_t timestamp3 = 0;
u_int32_t timestamp4 = 0;
u_int32_t timestamp10 = 0;
u_int32_t timestamp11 = 0;
u_int32_t timestamp12 = 0;
u_int32_t timestamp13 = 0;
u_int32_t timestamp14 = 0;
u_int32_t timestamp15 = 0;
u_int32_t timestamp16 = 0;
u_int32_t timestamp17 = 0;

bool printMotorData = false;
int speed = 270;
float limit = 4;
int pos = 250;
int delaySpeed;
int vinValue = 0;
int count = 0;
int mode;
float p = 3.1415926;
int velLimit = 3000;
bool printMon = true;
int currentSector, _currentSector, previousSector, difference, lastSector, fault, dutyCycle;
bool count1, count2, count3, count4;
String message4, lastMessage4, data;
char input4;
bool velCCW = false;
bool velCW = false;
bool positivePos = false;
bool negativePos = false;
bool negativePosDone = false;
bool positivePosDone = false;
bool stop = false;
bool highSpeed = false;
bool highGear = false;
bool stopDone = false;
int sensitivity = 100;
// const int freq = 15000;
// const int resolution = 13;
BLDCMotor motor = BLDCMotor(1); // 3
BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, enable);
HallSensor sensor = HallSensor(encApin, encBpin, encCpin, 1); // 3
int a1, b1, c1 = 0;
float setpoint;
bool startScript;
int scriptStage = 0;
float input, error, ITerm, kp, ki, dt, kd, outMax, outMin, dInput, lastInput, output;
// Interrupt routine intialisation
// channel A and B callbacks
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }
// put function declarations here:

void resetPIDs()
{
  motor.P_angle.timestamp_prev = 0;
  motor.P_angle.error_prev = 0;
  motor.P_angle.integral_prev = 0;
  motor.P_angle.output_prev = 0;

  motor.PID_velocity.timestamp_prev = 0;
  motor.PID_velocity.error_prev = 0;
  motor.PID_velocity.integral_prev = 0;
  motor.PID_velocity.output_prev = 0;

  motor.PID_current_q.timestamp_prev = 0;
  motor.PID_current_q.error_prev = 0;
  motor.PID_current_q.integral_prev = 0;
  motor.PID_current_q.output_prev = 0;

  motor.PID_current_d.timestamp_prev = 0;
  motor.PID_current_d.error_prev = 0;
  motor.PID_current_d.integral_prev = 0;
  motor.PID_current_d.output_prev = 0;
}

void stopMotor()
{
  velCCW = false;
  velCW = false;
  negativePos = false;
  positivePos = false;
  stop = true;
  count = 0;
  char cmd;
  // int status = std::stof(cmd);
  //  motor.move(0);
}

void velocityCW()
{
  resetPIDs();
  motor.PID_velocity.limit = 3000;
  motor.velocity_limit = 1500;
  motor.controller = MotionControlType::velocity;
  // motor.zero_electric_angle = initialAngle - 0.2;
  motor.zero_electric_angle = initialAngle;
  velCW = true;
  motor.enable();
}

void velocityCCW()
{
  resetPIDs();
  motor.PID_velocity.limit = 3000;
  motor.velocity_limit = 1500;
  motor.controller = MotionControlType::velocity;
  motor.zero_electric_angle = initialAngle;
  velCCW = true;
  motor.enable();
  // motor.move(speed * -1);
}

void positivePositioning(String data)
{
  serial_hw.println(data.toInt());
  if ((data.toInt() > 0) and not negativePos)
  {
    motor.disable();
    resetPIDs();
    motor.PID_velocity.limit = (float(speed)) / 180;
    // motor.velocity_limit = 1;
    motor.P_angle.P = 10;
    motor.controller = MotionControlType::angle;
    pos = data.toInt();
    count = 0;
    initialPos = motor.shaft_angle;
    setpoint = pos + initialPos;
    serial_hw.print("positive position is: ");
    serial_hw.println(pos);
    positivePos = true;
    motor.enable();
  }
}

void negativePositioning(String data)
{
  serial_hw.println(data.toInt());
  if ((data.toInt() > 0) and not positivePos)
  {
    motor.disable();
    resetPIDs();
    // motor.velocity_limit = 15;
    motor.PID_velocity.limit = (float(speed)) / 180;
    motor.P_angle.P = 10;
    motor.controller = MotionControlType::angle;
    pos = data.toInt();
    count = 0;
    initialPos = motor.shaft_angle;
    setpoint = initialPos - pos;
    serial_hw.print("negative position is: ");
    serial_hw.println(pos);
    negativePos = true;
    motor.enable();
  }
}

void continuousUpdate()
{
  if (printMon)
  {
    if ((millis() - timestamp1) > 10)
    {
      if (printMotorData)
      {
        timestamp1 = millis();
        // serial_hw.print(speed);
        // serial_hw.print("\t");
        serial_hw.print(motor.shaft_angle);
        serial_hw.print("\t");
        serial_hw.print(motor.shaft_velocity);
        serial_hw.print("\t");
        serial_hw.println(motor.voltage_limit);
        // serial_hw.print("\t");
        // serial_hw.println(encoderAngle, 3);
      }
    }

    if (millis() - timestamp2 > 100)
    {
      timestamp2 = millis();
      double sense1 = (double(double(analogRead(sensor1_adc)) / 4095)) * 3.3;
      double sense2 = (double(double(analogRead(sensor2_adc)) / 4095)) * 3.3;
      // serial_hw.print(sense1);
      // serial_hw.print("\t");
      // serial_hw.println(sense2);
      //  tft.setTextWrap(false);
      //  tft.fillScreen(ST77XX_BLACK);
      //  tft.setCursor(0, 30);
      //  tft.setTextColor(ST77XX_RED);
      //  tft.setTextSize(2);
      //  tft.print("Position    ");
      //  tft.println(motor.shaft_angle);
      //  tft.setTextColor(ST77XX_YELLOW);
      //  tft.setTextSize(2);
      //  tft.print("Velocity    ");
      //  tft.println(motor.shaft_velocity);
      //  tft.setTextColor(ST77XX_GREEN);
      //  tft.setTextSize(2);
      //  tft.print("current     ");
      //  tft.println(motor.voltage_limit);
    }
  }
  if (serial_hw.available())
  {
    input4 = serial_hw.read();
    message4 += input4;
    // serial_hw.print(input4);
  }
  if ((input4 == '\n'))
  {
    input4 = 0;
    lastMessage4 = message4;
    message4 = ""; //  ESP.restart();דפ

    if (lastMessage4.indexOf("reset") >= 0)
    {
      serial_hw.println("restarting ESP");
      NVIC_SystemReset();
    }
    if (lastMessage4.indexOf("print") >= 0)
    {
      serial_hw.println("printing..");
      printMon = !printMon;
    }
    if (lastMessage4.indexOf("right") >= 0)
    {
      serial_hw.println("positive revolution");
      velocityCW();
    }

    if (lastMessage4.indexOf("left") >= 0)
    {
      serial_hw.println("negative revolution");
      velocityCCW();
    }

    if (lastMessage4.indexOf("speed") >= 0)
    {
      serial_hw.println("setting speed");
      data = lastMessage4.substring(5);
      serial_hw.println(data);
      serial_hw.println(data.toInt());
      if (data.toInt() > 0)
      {
        speed = data.toInt();
        serial_hw.print("speed is: ");
        serial_hw.println(speed);
      }
    }

    if (lastMessage4.indexOf("limit") >= 0)
    {
      serial_hw.println("setting limit");
      data = lastMessage4.substring(5);
      serial_hw.println(data);
      serial_hw.println(data.toInt());
      if (data.toInt() > 0)
      {
        limit = data.toFloat();
        serial_hw.print("limit is: ");
        serial_hw.println(limit);
        motor.voltage_limit = limit;
        motor.PID_velocity.limit = limit;
        //  motor.init();
        //  motor.P_angle.limit = velLimit;
        // motor.disable();
      }
    }

    if (lastMessage4.indexOf("posi") >= 0)
    {
      serial_hw.println("positive positioning");
      data = lastMessage4.substring(4);
      positivePositioning(data);
      //  serial_hw.println(data);
    }
    if (lastMessage4.indexOf("nega") >= 0)
    {
      serial_hw.println("negative positioning");
      data = lastMessage4.substring(4);
      negativePositioning(data);
      // serial_hw.println(data);
    }

    if (lastMessage4.indexOf("teachp1") >= 0)
    {
      serial_hw.println("teachP1 ");
      // positivePositioning(data);
      positionP1 = motor.shaft_angle;
      serial_hw.print("positionP1 is: ");
      serial_hw.println(positionP1);
      serial_hw.println();
    }

    if (lastMessage4.indexOf("teachp2") >= 0)
    {
      serial_hw.println("teachP2 ");
      // positivePositioning(data);
      positionP2 = motor.shaft_angle;
      serial_hw.print("positionP2 is: ");
      serial_hw.println(positionP2);
      serial_hw.println();
    }

    if (lastMessage4.indexOf("stp") >= 0)
    {
      serial_hw.println("stopping motor");
      stopMotor();
    }

    if (lastMessage4.indexOf("startScript") >= 0)
    {
      motor.disable();
      resetPIDs();
      scriptStage = 0;
      motor.PID_velocity.limit = (float(speed)) / 180;
      motor.controller = MotionControlType::angle;
      motor.enable();
      serial_hw.println("starting script");
      serial_hw.print("P1 ==>");
      serial_hw.println(positionP1);
      serial_hw.print("P2 ==>");
      serial_hw.println(positionP2);
      initialPos = sensor.getAngle();
      serial_hw.print("current angle");
      serial_hw.println(initialPos);
      startScript = true;
    }
    if (lastMessage4.indexOf("stopScript") >= 0)
    {
      serial_hw.println("stopping script");
      startScript = false;
      scriptStage = 0;
      serial_hw.println("stopping motor");
      stopMotor();
    }

    if (lastMessage4.indexOf("mode") >= 0)
    {
      serial_hw.println("mode");
      data = lastMessage4.substring(2);
      serial_hw.println(data.toInt());
      if (data.toInt() > 0)
      {
        mode = data.toInt();
        char binary[3] = {0}; // This is where the binary representation will be stored
        itoa(mode, binary, 2);
        if (binary[0] == 0 or binary[0] == NULL)
        {
          // digitalWrite(mode0, LOW);
        }
        else
        {
          // digitalWrite(mode0, HIGH);
        }
        if (binary[1] == 0 or binary[1] == NULL)
        {
          // digitalWrite(mode1, LOW);
        }
        else
        {
          // digitalWrite(mode1, HIGH);
        }
        if (binary[2] == 0 or binary[2] == NULL)
        {
          // digitalWrite(mode2, LOW);
        }
        else
        {
          // digitalWrite(mode2, HIGH);
        }
      }
    }
  }
}

void FOCsetup()
{
  motor.linkSensor(&sensor);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12;
  driver.pwm_frequency = 32000;
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 1;
  // set motion control loop to be used
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::velocity;
  // contoller configuration
  // default parameters in defaults.h
  // velocity PI controller parameters
  motor.PID_velocity.P = 0.004; // was 1.4
  motor.PID_velocity.I = 0.1;   // was 2
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 12;   // 4
  motor.current_limit = 0.95; // 2
  motor.velocity_limit = 1500;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.limit = 3000;
  motor.PID_velocity.output_ramp = 1000;
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.02f;
  motor.useMonitoring(serial_hw);
  // use monitoring with serial
  // comment out if not needed
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
  initialAngle = motor.zero_electric_angle;
}

void testdrawtext(char *text, uint16_t color)
{
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void testlines(uint16_t color)
{
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(0, 0, x, tft.height() - 1, color);
    delay(0);
  }
  for (int16_t y = 0; y < tft.height(); y += 6)
  {
    tft.drawLine(0, 0, tft.width() - 1, y, color);
    delay(0);
  }

  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(tft.width() - 1, 0, x, tft.height() - 1, color);
    delay(0);
  }
  for (int16_t y = 0; y < tft.height(); y += 6)
  {
    tft.drawLine(tft.width() - 1, 0, 0, y, color);
    delay(0);
  }

  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(0, tft.height() - 1, x, 0, color);
    delay(0);
  }
  for (int16_t y = 0; y < tft.height(); y += 6)
  {
    tft.drawLine(0, tft.height() - 1, tft.width() - 1, y, color);
    delay(0);
  }

  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(tft.width() - 1, tft.height() - 1, x, 0, color);
    delay(0);
  }
  for (int16_t y = 0; y < tft.height(); y += 6)
  {
    tft.drawLine(tft.width() - 1, tft.height() - 1, 0, y, color);
    delay(0);
  }
}

void tftPrintTest()
{
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.println("Hello Endymed!");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello Endymed!");
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("Hello Endymed!");
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(100);
  tft.setCursor(0, 0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print(millis() / 100);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(" seconds.");
}

void testdrawrects(uint16_t color)
{
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, color);
  }
}

void testfillrects(uint16_t color1, uint16_t color2)
{
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = tft.width() - 1; x > 6; x -= 6)
  {
    tft.fillRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, color1);
    tft.drawRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, color2);
  }
}

void handle_touch()
{
  // touch.begin(72U, &I2C_main);
  uint16_t x, y, z1, z2;
  // if (touch.read_touch(&x, &y, &z1, &z2))
  // {
  //   serial_hw.print("Touch point: (");
  //   // //  if (z1 > sensitivity)
  //   //   {
  //   serial_hw.print(x);
  //   serial_hw.print(" / ");
  //   serial_hw.print(y);
  //   serial_hw.println(")");
  // }
  // I2C_main.end();
}

void testSD2()
{
  File myFile;
  SD.setDx(PC8, PC9, PC10, PC11);
  SD.setCMD(PC12);
  SD.setCK(PD2); // using PinName
  SD.begin();
  if (SD.exists("example.txt"))
  {
    serial_hw.println("example.txt exists.");
  }
  else
  {
    serial_hw.println("example.txt doesn't exist.");
  }

  // open a new file and immediately close it:
  serial_hw.println("Creating example.txt...");
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.close();

  // Check to see if the file exists:
  if (SD.exists("example.txt"))
  {
    serial_hw.println("example.txt exists.");
  }
  else
  {
    serial_hw.println("example.txt doesn't exist.");
  }
}

void testSD()
{
  uint32_t file_size = 0, seek_val = false, peek_val = 0;
  uint32_t byteswritten, bytesread = 0;
  /* File write buffer */
  uint8_t wtext[] = "This is the Arduino SD Test working with FatFs.";
  /* File read buffer */
  uint8_t rtext[200];
  uint32_t i = 0;
  bool isdir = false;
  File MyFile;

  delay(100);

  /* Test mkdir() method */
  serial_hw.print("Creating 'STM32' directory...");
  if (SD.mkdir("STM32"))
  {
    serial_hw.println("OK");
  }
  else
  {
    serial_hw.println("KO");
  }
  serial_hw.print("Creating 'ARDUINO' directory...");
  if (SD.mkdir("ARDUINO"))
  {
    serial_hw.println("OK");
  }
  else
  {
    serial_hw.println("KO");
  }
  serial_hw.print("Creating 'ARDUINO/SD' directory...");
  if (SD.mkdir("ARDUINO/SD"))
  {
    serial_hw.println("OK");
  }
  else
  {
    serial_hw.println("KO");
  }
}
void setup()
{
  // pinMode(M_PWM, OUTPUT);
  // digitalWrite(M_PWM, HIGH);
  // delay(3000);
  analogReadResolution(12);
  pinMode(touch_interrupt_pin, INPUT_PULLUP);
  attachInterrupt(touch_interrupt_pin, handle_touch, FALLING);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(enable, OUTPUT);
  serial_hw.begin(500000);
  sensor.pullup = Pullup::USE_EXTERN;
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  FOCsetup();
  serial_hw.println(F_CPU);
  delay(1000);

  serial_hw.println("Adafruit Serial Flash Info example");
  flash.begin();

  serial_hw.print("JEDEC ID: 0x");
  serial_hw.println(flash.getJEDECID(), HEX);
  serial_hw.print("Flash size: ");
  serial_hw.print(flash.size() / 1024);
  serial_hw.println(" KB");

  motor.enable();
  // tft.invertDisplay(true);
  tft.init(240, 320);
  tft.setRotation(2);
  // tft.invertDisplay(true);
  tft.fillScreen(ST77XX_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST77XX_WHITE);
  delay(1000);
  tftPrintTest();
  // if (!touch.begin(72U, &I2C_main))
  // {
  //   serial_hw.println("Couldn't find touch controller");
  //   while (1)
  //     delay(10);
  // }
  // serial_hw.println("Found touch controller");
  // delay(10);
  bool disp = false;
  serial_hw.print("\nInitializing SD card...");
  card.setDx(PC8, PC9, PC10, PC11);
  card.setCK(PC12);
  card.setCMD(PD2);
  while (!card.init())
  {
    if (!disp)
    {
      serial_hw.println("initialization failed. Is a card inserted?");
      disp = true;
    }
    delay(10);
  }

  serial_hw.println("A card is present.");
  testSD2();

  if (!fatFs.init())
  {
    serial_hw.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return;
  }

  // print the type of card
  serial_hw.print("\nCard type: ");
  switch (card.type())
  {
  case SD_CARD_TYPE_SD1:
    serial_hw.println("SD1");
    break;
  case SD_CARD_TYPE_SD2:
    serial_hw.println("SD2");
    break;
  case SD_CARD_TYPE_SDHC:
    serial_hw.println("SDHC");
    break;
  default:
    serial_hw.println("Unknown");
  }

  // print the type and size of the first FAT-type volume
  uint64_t volumesize;
  serial_hw.print("\nVolume type is FAT");
  serial_hw.println(fatFs.fatType(), DEC);
  serial_hw.println();

  volumesize = fatFs.blocksPerCluster(); // clusters are collections of blocks
  volumesize *= fatFs.clusterCount();    // we'll have a lot of clusters
  volumesize *= 512;                     // SD card blocks are always 512 bytes
  serial_hw.print("Volume size (bytes): ");
  serial_hw.println(volumesize);
  serial_hw.print("Volume size (Kbytes): ");
  volumesize /= 1024;
  serial_hw.println(volumesize);
  serial_hw.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  serial_hw.println(volumesize);
  // I2C_main.end();
  // delay(300);
  // tft.fillScreen(ST77XX_BLACK);
  // testlines(ST77XX_WHITE);
  // delay(500);
  // testdrawrects(ST77XX_GREEN);
  // delay(500);
  // testfillrects(ST77XX_YELLOW, ST77XX_MAGENTA);
  // delay(500);
}

void loop()
{
  // put your main code here, to run repeatedly:
  continuousUpdate();
  // hallAngle = sensor.getAngle();--------------------
  motor.loopFOC();
  // motor.move(-95);
  // motor.monitor();
  // esp_task_wdt_init(30, false); // disable WDT idle task
  // delayMicroseconds(200);
  // Serial.println(_currentSector);
  // updateHalls();
  // driveMotor();
  // delaySpeed = 1 / speed;
  if (velCW)
  {
    motor.move(speed);
  }
  if (velCCW)
  {
    motor.move(speed * -1);
  }
  if (negativePos)
  {
    motor.move(setpoint);
    // if (((hallAngle - initialPos) + pos) < 9)
    if ((motor.shaft_angle - setpoint) < window)
    {
      motor.move(0);
      motor.disable();
      negativePos = false;
      serial_hw.println("negativePosDone");
    }
  }
  if (positivePos)
  {
    motor.move(setpoint);
    // if ((pos - (hallAngle - initialPos)) < 9)
    if ((setpoint - motor.shaft_angle) < window)
    {
      motor.move(0);
      motor.disable();
      positivePos = false;
      serial_hw.println("positivePosDone");
    }
  }
  if (stop)
  {
    motor.move(0);
    motor.disable();
    if ((motor.shaft_velocity <= 0.1) and (motor.shaft_velocity >= -0.1))
    {
      stop = false;
      motor.disable();
      serial_hw.println("stopped");
      resetPIDs();
    }
  }
  // if ((micros() - timestamp4) > 10)
  // {
  //  timestamp4 = micros();
  if (startScript)
  {
    switch (scriptStage)
    {
    case 0:
      // initialPos = sensor.getAngle();
      if (motor.shaft_angle > positionP1)
      {
        scriptStage = 1;
        serial_hw.println("going to position P1 negatively");
      }
      else
      {
        serial_hw.println("going to position P1 positively");
        scriptStage = 10;
      }
      break;

    case 1:
      motor.PID_velocity.limit = (float(speed)) / 180;
      motor.move(positionP1);
      if ((millis() - timestamp11) > 1)
      {
        timestamp11 = millis();
        if ((motor.shaft_angle - positionP1) <= window)
        {
          scriptStage = 3;
        }
      }
      break;

    case 10:
      motor.PID_velocity.limit = (float(speed)) / 180;
      motor.move(positionP1);
      if ((millis() - timestamp12) > 1)
      {
        timestamp12 = millis();
        if ((positionP1 - motor.shaft_angle) <= window)
        {
          scriptStage = 3;
        }
      }
      break;

    case 3:
      if (motor.shaft_angle < positionP2)
      {
        scriptStage = 4;
        serial_hw.println("going to position P2 positively");
      }
      else
      {
        scriptStage = 40;
        serial_hw.println("going to position P2 negatively");
      }
      break;

    case 4:
      motor.PID_velocity.limit = (float(speed)) / 180;
      motor.move(positionP2);
      // motor.enable();
      if ((millis() - timestamp15) > 1)
      {
        timestamp15 = millis();
        if ((positionP2 - motor.shaft_angle) <= window)
        {
          scriptStage = 0;
        }
      }
      break;

    case 40:
      motor.PID_velocity.limit = (float(speed)) / 180;
      motor.move(positionP2);
      if ((millis() - timestamp16) > 1)
      {
        timestamp16 = millis();
        if ((motor.shaft_angle - positionP2) <= window)
        {
          scriptStage = 0;
        }
      }
      break;
    }
    // }
  }
}

// put function definitions here:
