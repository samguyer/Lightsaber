#include <adel.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <DFPlayer_Mini_Mp3.h>
#include <Adafruit_LSM9DS0.h>

// -- Saber state --------------------------

bool g_saber_on = false;

// -- Button -------------------------------

#define BUTTON_PIN 4

// -- Blade --------------------------------

#define NUM_SEGMENTS 10

// -- PCA9685 driver
//    Called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define TOTAL_LENGTH 1200
#define SEGMENT_LENGTH (TOTAL_LENGTH / NUM_SEGMENTS)

#define TIME_TO_ACTIVATE 600
#define TIME_TO_DEACTIVATE 600

unsigned int g_max_brightness = 4095;

// -- Gyroscope ----------------------------

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

typedef enum { NONE, UP, DOWN, LEFT, RIGHT } Direction;

// -- Sounds -------------------------------

#define SABER_ON_SOUND 1
#define SABER_OFF_SOUND 2
#define LONG_HUM_SOUND 3
#define MEDIUM_HUM_SOUND 4
#define DOUBLE_HUM_SOUND 5
#define SHORT_HUM_SOUND 6
#define SWING_1_SOUND 7
#define SWING_2_SOUND 8
#define SWING_3_SOUND 9

// -----------------------------------------------

// -- Render the saber
//    Length is a number between 0 and 1000
void render(int len)
{
  int segments_on = len / SEGMENT_LENGTH;
  int partial = len - segments_on * SEGMENT_LENGTH;

  for (int i = 0; i < NUM_SEGMENTS; i++) {
    if (i < segments_on) {
      pwm.setPWM(i, 0, g_max_brightness );
      // analogWrite(SEG_PIN[i], 255);
    }
    if (i == segments_on) {
      int level = map(partial/2, 0, SEGMENT_LENGTH, 0, g_max_brightness);
      pwm.setPWM(i, 0, level );
      // analogWrite(SEG_PIN[i], dim8_video(partial));
    }
    if (i > segments_on) {
      pwm.setPWM(i, 0, 0 );
      // analogWrite(SEG_PIN[i], 0);
    }
  }
}

adel waitbutton(int pin)
{
   abegin:
   while (1) {
      await (digitalRead(pin) == HIGH);
      adelay (50);
      if (digitalRead(pin) == HIGH) {
         await (digitalRead(pin) == LOW);
         afinish;
      }
   }
   aend;
}

int curlevel = 0;
long saber_start = 0;

adel saber_on()
{
  abegin:

  saber_start = millis();
  while (millis() - saber_start < TIME_TO_ACTIVATE) {
    curlevel = map(millis() - saber_start, 0, TIME_TO_ACTIVATE, 0, TOTAL_LENGTH);
    render(curlevel);
    adelay(10);
  }

  render(TOTAL_LENGTH);

  aend;
}

adel saber_off()
{
  abegin:

  saber_start = millis();
  while (millis() - saber_start < TIME_TO_DEACTIVATE) {
    curlevel = map(millis() - saber_start, 0, TIME_TO_DEACTIVATE, TOTAL_LENGTH, 0);
    render(curlevel);
    adelay(10);
  }

  render(0);

  aend;
}

bool get_swing(Direction& dir)
{
  lsm.read();

  float gyroY = ((float) lsm.gyroData.y) / 4000.0;
  float gyroZ = ((float) lsm.gyroData.z) / 4000.0;

  dir = NONE;

  float mag = sqrt(gyroY * gyroY + gyroZ * gyroZ);

  if (mag < 3.0) {
    dir = NONE;
    return false;
  } else {
    if (abs(gyroY) > abs(gyroZ)) {
      // -- Mostly up-down
      if (gyroY < 0.0) 
        dir = UP;
      else 
        dir = DOWN;
    } else {
      // -- Mostly left-right
      if (gyroZ < 0.0) 
        dir = RIGHT;
      else 
        dir = LEFT;
    }

    return true;
  }
}

adel motion()
{
  avars {
    int sound;
    Direction dir;
  }
  
  abegin:

  while (1) {
    if (g_saber_on) {
      my(sound) = 0;
      if (get_swing(my(dir))) {
        if (my(dir) == UP)    my(sound) = SWING_1_SOUND;
        if (my(dir) == DOWN)  my(sound) = SWING_1_SOUND;
        if (my(dir) == LEFT)  my(sound) = SWING_3_SOUND;
        if (my(dir) == RIGHT) my(sound) = SWING_2_SOUND;
      }

      if (my(sound) != 0) {
        mp3_play(my(sound));
        // -- Wait for the sound to finish before starting a new one
        adelay(800);
      }
    }

    // my(sound) = random(MEDIUM_HUM_SOUND, SHORT_HUM_SOUND+1);

    adelay(10);
  }
  
  aend;
}

adel onoff()
{
  abegin:

  while (1) {
    andthen( waitbutton(BUTTON_PIN) );
    // andthen( trigger_sound(SABER_ON_SOUND) );
    mp3_play(SABER_ON_SOUND);
    adelay(200);
    andthen( saber_on() );
    adelay(500);
    mp3_play(LONG_HUM_SOUND);
    g_saber_on = true;
    andthen( waitbutton(BUTTON_PIN) );
    g_saber_on = false;
    // andthen( trigger_sound(SABER_OFF_SOUND) );
    mp3_play(SABER_OFF_SOUND);
    andthen( saber_off() );
  }
  
  aend;
}

adel saber()
{
  abegin:
  atogether( onoff(), motion() );
  aend;
}

// -----------------------------------------------
//    Set up
// -----------------------------------------------

void setup()
{
  delay(500);
  Serial.begin (9600);

  // -- Set up the DFPlayer
  delay(200);
  mp3_set_serial (Serial);  //set Serial for DFPlayer-mini mp3 module
  delay(50);
  // mp3_DAC(false);
  // mp3_reset();
  mp3_set_volume (30);
  // delay(50);
  // mp3_set_EQ (5);  //0~5
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, LOW);

  // -- Set up the gyroscope
  if (!lsm.begin())
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);

  // -- Set up the PCA9685 PWM module  
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  
  render(0);
}

// -----------------------------------------------
//   Main loop
// -----------------------------------------------

void loop()
{
  arepeat( saber() );
}

