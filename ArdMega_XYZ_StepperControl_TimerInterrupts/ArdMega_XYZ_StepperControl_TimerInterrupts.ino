//-----------------------------CONTROL MODES-------------------------------------------------------------------------//
#define STP 0
#define MAN 1
#define TRAJ 2
byte control_mode = 0;

//----------------------------------Trajectory Parameters------------------------------------------------------------//
bool pos_init = 0;
double curr_x, curr_y, curr_z, trg_x, trg_y, trg_z; //Current and target positions in mm (x,y,z,)
double tf, ta;  //Total trajectory time and acceleration/deceleration time
#define vec_length     100
volatile int m1_steps[vec_length];  //Array containing steps per each time interval (z axis)
volatile int m2_steps[vec_length];  //Array containing steps per each time interval (th1)
volatile int m3_steps[vec_length];  //Array containing steps per each time interval (th2)
//Minimum step size for arm joints (in rad)
double min_step = 0.000063; //(360/steps_p_rev)*(pi/180)
//(360/100,000)*(pi/180) = 0.000063

//Minimum step size for the ball screw (in mm)
double step_size_BS = 5.00 / 1000.0; //(pitch / steps_p_rev)
double ti = tf / vec_length;
#define clock_freq  16000000
#define prescaler  64
//#define L 200
#define ppr 100000  //Pulses per revolution
volatile unsigned int interval = (2 * clock_freq*ti) / prescaler;
int acc_time = (int)(ta / ti);
const float c1 = ppr / (2 * PI * 1000000);
const float c_xy = 2 * PI / ppr;  //Constant for XY motors
const float c_z = 2 * PI / 1000;  //Constant for Z motor
//-------------------------------Check Variables-----------------------------------------------------------------------//
//Variables uses for checking purposes: check the number of steps that the trajectory performed, and the distance that it moved
int total_steps_x = 0, total_steps_y = 0;
double distance_x, distance_y;
// ---------------------------MOTORS VARIABLES AND FUNCTIONS----------------------------------------------------------//
//Motor Pins
#define NUM_STEPPERS    3

struct stepperInfo {
  volatile int id;      //Motor ID: th1(0), th2(1), z(2)
  volatile unsigned int t_delay;  //delay between steps (units: clock counts)
  volatile unsigned int move_steps = 0; //number of steps to move
  volatile unsigned int go2_steps;  //target motor position (in steps)
  volatile int curr_steps;      //current motor position (in steps)
  volatile unsigned int count;   //current count of steps
  volatile int dir;
  volatile double pos = 0;
  volatile bool pulse_status = false;
  volatile bool motor_status = false;
  volatile bool finished = true;
  volatile void (*toggleFunc)();
  volatile void (*dirFunc)(int);
};

volatile stepperInfo steppers[NUM_STEPPERS];

#define TH1_DIR_PIN     22 //PA0
#define TH1_STEP_PIN    23 //PA1
#define TH2_DIR_PIN     24  //PA2
#define TH2_STEP_PIN    25  //PA3
#define Z_DIR_PIN       26  //PA4
#define Z_STEP_PIN      27  //PA5

//Timer On/Off macros (Timer1 compare interrupt)
#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);
#define TIMER3_INTERRUPTS_ON    TIMSK3 |=  (1 << OCIE3A);
#define TIMER3_INTERRUPTS_OFF   TIMSK3 &= ~(1 << OCIE3A);
#define TIMER4_INTERRUPTS_ON    TIMSK4 |=  (1 << OCIE4A);
#define TIMER4_INTERRUPTS_OFF   TIMSK4 &= ~(1 << OCIE4A);

//Direct port register write functions
#define TH1_TOOGLE_STEP     PORTA ^=(1<<1); //Toogle PortA bit1 (pin 23)
#define TH2_TOOGLE_STEP     PORTA ^=(1<<3); //Toogle PortA bit3 (pin 25)
#define Z_TOOGLE_STEP       PORTA ^=(1<<5); //Toogle PortA bit5 (pin 27)

void th1_toggle_func() {
  TH1_TOOGLE_STEP;
}
void th2_toggle_func() {
  TH2_TOOGLE_STEP;
}
void z_toggle_func() {
  Z_TOOGLE_STEP;
}
//Set Direction
#define TH1_DIR_CW    PORTA |=  (1 << 0); //Set bit (pin 22)
#define TH1_DIR_CCW   PORTA &= ~(1 << 0); //Clear bit (pin 22)
#define TH2_DIR_CW    PORTA |=  (1 << 2); //Set bit (pin 24)
#define TH2_DIR_CCW   PORTA &= ~(1 << 2); //Clear bit (pin 24)
#define Z_DIR_UP      PORTA |=  (1 << 4); //Set bit (pin 26)
#define Z_DIR_DOWN    PORTA &= ~(1 << 4); //Clear bit (pin 26)
//----------------------------------------------------------------------------------------------------------//

volatile unsigned int count1=0, count2 = 0, count3 = 0;
volatile byte i1, i3, i2;
volatile int rem_steps1, rem_steps2, rem_steps3;
volatile bool state1, state2, state3;

String inString = "";

void setup() {
  // Configure Motor Pins:
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(TH1_STEP_PIN, OUTPUT);
  pinMode(TH1_DIR_PIN, OUTPUT);
  pinMode(TH2_STEP_PIN, OUTPUT);
  pinMode(TH2_DIR_PIN, OUTPUT);

  //Initialise Serial Port:
  Serial.begin(115200);
  //Reserves bytes in memory for string manipulation
  //inString.reserve(30); //number of bytes to reserve
  //Configure Timers:
  noInterrupts();
  //Setup Timer1 Interrupt
  TCCR1A = 0; //Reset Timer1
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 1000;                         // Load compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    //Prescaler: CSx2(0), CSx1(1), CSx0(1) --> 64

  //Setup Timer3 Interrupt
  TCCR3A = 0; //Reset Timer1
  TCCR3B = 0;
  TCNT3  = 0;
  OCR3A = 1000;                         // Load compare value
  TCCR3B |= (1 << WGM32);                   // CTC mode
  TCCR3B |= ((1 << CS31) | (1 << CS30));;
  //Setup Timer4 Interrupt
  TCCR4A = 0; //Reset Timer1
  TCCR4B = 0;
  TCNT4  = 0;
  OCR4A = 1000;                         // Load compare value
  TCCR4B |= (1 << WGM42);                   // CTC mode
  TCCR4B |= ((1 << CS41) | (1 << CS40));;
  interrupts();

  //Instanciate motor variables:
  steppers[0].id = 0;
  steppers[0].t_delay = 1000;
  steppers[0].curr_steps = 0;
  //steppers[0].dirFunc = th1_Dir;

  steppers[1].id = 1;
  steppers[1].t_delay = 1000;
  steppers[1].curr_steps = 0;
  //steppers[1].dirFunc = th2_Dir;

  steppers[2].id = 2;
  steppers[2].t_delay = 1000;
  steppers[2].curr_steps = 0;
  //steppers[2].dirFunc = z_Dir;
}

void loop() {
  // Wait for a serial command:
  while (Serial.available() == 0);
  //Read serial command:
  if (Serial.available()) {
    inString = Serial.readStringUntil('\n');
  }
  readCommand();


  initialiseInterrupts();
  waitUntilFinished();

  if (control_mode == TRAJ) {
    String serialOut = "T:" + String(steppers[0].pos) + ";" + String(steppers[1].pos) + ";" + String(steppers[2].pos) + ";";
    Serial.println(serialOut);
    curr_x = 200*cos(steppers[1].pos) + 200*cos(steppers[1].pos + steppers[2].pos);
    curr_y = 200*sin(steppers[1].pos) + 200*sin(steppers[1].pos + steppers[2].pos);
    
    curr_z = trg_z;
  }

  String serialOut = String(steppers[0].curr_steps) + "," + String(steppers[1].curr_steps) + "," + String(steppers[2].curr_steps) + ",";
  Serial.println(serialOut);
}
