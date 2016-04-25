
/*
 * rosserial Uarm command
 *
 * This sketch is control the Uarm using ROS and the arduiono
 * 
 * Authors : Alexis MARTIN and Dario PALMA 
 *
 * For more information on the code
 * Email :
 * alexis.francois.martin@gmail.com
 */

//**********************************************************************
// Arduino setup
//**********************************************************************
//#if (ARDUINO >= 100)
//  #include <Arduino.h>
//#else
//  #include <WProgram.h>
//#endif

//**********************************************************************
// Arduino libraries include
//**********************************************************************
#include <Servo.h> 
#include <MsTimer2.h> 
#include <ax12.h>
#include <Ultrasonic.h>
#include "Uarm.h"

//**********************************************************************
// Ros include
//**********************************************************************
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

//**********************************************************************
// Arduino setup
//**********************************************************************
#define baudrate  115200 //57600 or 115200

#define SMParasolpin 44
#define SMGripperpin 42
#define SM0pin 32
#define SM1pin 34
#define SM2pin 36
#define SM3pin 40

#define US1pin 46
#define US2pin 50

#define servospeed 300 //350 //°/s (datasheet -> 0.17s/60°) 300 (pour la marge de sécu) 100 (test superSLOW)

#define UNDEFINED 0
#define CLOSED 1
#define OPEN 2

#define rotSpeed 60 //from 0 to 114 rpm
#define torqueLIM 600 
#define openMaxAngle -50 // from -150 to 150°
#define openAngle -10
#define closeAngle 65 // from -150 to 150°
#define middleAngle 20 // from -150 to 150°
#define balanceAngle 10

#define positionAlerte 60 //en cm

//**********************************************************************
// Uarm dimensions 
//**********************************************************************
const float L1= 140; //first joint lenght (mm) //137
const float L2= 152; //second joint lenght (mm) //150
const float Ea= 20.0;  // exentricity of the arm (mm)
const float Eb= 20.0;  // tangent exentricity of the arm (mm)
const float Dx= 0.0;  // exentricity of the robot base along x (mm)
const float Dy= 0.0;  // exentricity of the robot base along y (mm)
const float Dz= 0.0;  // exentricity of the robot base along z (mm)

  // SERVO 0 data : qServo = q0nlincoef*qModel + q0offset
const int q0servmin= 14;              // limite min de consigne pour serv0 en °
const int q0servmax= 154;             // limite max de consigne pour serv0 en °
const int q0servmarg = 1;             // marge de consigne pour serv0 en °
const float q0modmin = 0.0;           // limite min de consigne pour serv0 en ° (selon modèle)
const float q0modmax = 178.5;         // limite max de consigne pour serv0 en ° (selon modèle)
const float q0nlincoef = 72.0/90.0;   // coefficient non linéarité entre serv0 et modèle en °
const float q0offset = 14.0;          // offset entre modèle et serv0 en °

  // SERVO 1 data : qServo = q1nlincoef*qModel + q1offset
const int q1servmin= 38;              // limite min de consigne pour serv1 en °
const int q1servmax= 135;             // limite max de consigne pour serv1 en °
const int q1servmarg = 1;             // marge de consigne pour serv1 en °
const float q1modmin = 7.0;           // limite min de consigne pour serv1 en ° (selon modèle)
const float q1modmax = 115.0;         // limite max de consigne pour serv1 en ° (selon modèle)
const float q1nlincoef = 74.0/83.0;   // coefficient non linéarité entre serv1 et modèle en °
const float q1offset = 32.0;          // offset entre modèle et serv1 en °

  // SERVO 2 data : qServo = -q2nlincoef*qModel + q2offset
const int q2servmin= 43;              // limite min de consigne pour serv2 en °
const int q2servmax= 146;             // limite max de consigne pour serv2 en °
const int q2servmarg = 1;             // marge de consigne pour serv2 en °
const float q2modmin = 72.0;          // limite min de consigne pour serv2 en ° (selon modèle)
const float q2modmax = 182.0;         // limite max de consigne pour serv2 en ° (selon modèle)
const float q2nlincoef = 17.0/18.0;   // coefficient non linéarité entre serv2 et modèle en °
const float q2offset = 215.0;         // offset entre modèle et serv2 en °

  // SERVO 3 data : qServo = q3nlincoef*qModel + q4offset
const int q3servmin= 10;              // limite min de consigne pour serv3 en °
const int q3servmax= 170;             // limite max de consigne pour serv3 en °
const int q3servmarg = 1;             // marge de consigne pour serv3 en °
const float q3modmin = -10.0;         // limite min de consigne pour serv3 en ° (selon modèle)
const float q3modmax = 170.0;          // limite max de consigne pour serv3 en ° (selon modèle)
const float q3nlincoef = -1.0;         // coefficient non linéarité entre serv3 et modèle en °
const float q3offset = 180;            // offset entre modèle et serv3 en °

  // SERVO Gripper data : 
const int qGrippermin= 66;              // limite min de consigne pour servGripper en °
const int qGrippermax= 157;             // limite max de consigne pour servGripper en °
const int qGrippermarg = 1;             // marge de consigne pour servGripper en °

//**********************************************************************
// Front Pliers variables
//**********************************************************************
AX12 motor[2] = {AX12(), AX12()};
char openP[]="open";
char openMaxP[] = "openMax";
char closeP[]="close";
char middle[]="middle";
char balanceP[] = "balance";
bool arretBalance;

//**********************************************************************
// Init Postion setup
//**********************************************************************
#define initSM0_angle 90        // position initiale du servo de base selon le modèle (en °)
#define initSM1_angle 70        // position initiale du servo du bras 1 selon le modèle (en °)
#define initSM2_angle 120       // position initiale du servo du bras 2 selon le modèle (en °)
#define initSM3_angle 90        // position initiale du servo de l'effecteur selon le modèle (en °)
#define initSMGripper_angle 120 // position initiale du servo de la pince (en °)
#define initSMParasol_angle 20        // position initiale du servo du parasol (en °)
#define endSMParasol_angle 90       // position finale du servo du parasol (en °)

// init position convert from ° to rad
float qinit[4]={float(float(initSM0_angle)*PI/180.0),float(float(initSM1_angle)*PI/180.0),float(float(initSM2_angle)*PI/180.0), float(float(initSM3_angle)*PI/180.0)}; 

//**********************************************************************
// Uarm variable
//**********************************************************************
volatile float X_current= 0;                // en mm
volatile float Y_current= 0;                // en mm
volatile float Z_current= 0;                // en mm
volatile float theta_current= 0;            // en rad
volatile float q0_command= qinit[0];        // en rad
volatile float q1_command= qinit[1];        // en rad
volatile float q2_command= qinit[2];        // en rad
volatile float q3_command= qinit[3];        // en rad
volatile float q0_current= qinit[0];        // en rad 
volatile float q1_current= qinit[1];        // en rad
volatile float q2_current= qinit[2];        // en rad
volatile float q3_current= qinit[3];        // en rad

volatile int gripper_state = UNDEFINED ;     // état de la pince

volatile int parasol_state = CLOSED;              // état du parasol (fermé = 0, ouvert = 1);

volatile boolean fresh_command = false;     // true if the command is "fresh" (receive but never use) / false otherwise
volatile int behaviour_uarm = 0;            // describe the current behaviour of the Uarm

//**********************************************************************
// Interpolation parameter
//**********************************************************************
int tf;
const float smvseuil= 5.0;  // seuil de smallmove exprimé en degré
const int Ti=25;            // période d'exécution de l'isr (en ms)
int coeffsec=2;             // coefficient de sécurité de la non prise en compte de la saturation de vitesse 
int step_TODO = -1;
int step_DONE = 0;
float angleGOAL[4];
float qi[4];
float D[4];

//**********************************************************************
// Timer and synchronisation parameters
//**********************************************************************
const int mainLoop_t= 10; // période de la boucle loop (en ms)

//**********************************************************************
// ROS wide variables
//**********************************************************************
ros::NodeHandle nh;
geometry_msgs::Twist current_state;
std_msgs::Int16 behaviour;
std_msgs::Int16 gripperState;
std_msgs::Int32 pos1;
std_msgs::Int32 pos2;

//**********************************************************************
// Arduino wide variables
//**********************************************************************
Servo serv0;
Servo serv1;
Servo serv2;
Servo serv3;
Servo servGripper;
Servo servParasol;
Uarm uarm(L1, L2, Ea, Eb, Dx, Dy, Dz);
Ultrasonic ultrasonicFront(US1pin);
Ultrasonic ultrasonicBack(US2pin);

//**********************************************************************
// Callback(s)
//**********************************************************************
// Uarm position callback (input position in mm and ° -> update qX_command in rad)
void PositionCommandCallBack(const geometry_msgs::Twist& cmd_msg){
  // disable all isr
  cli();
  
  //copy the command
  float state[4];
  state[0]= cmd_msg.linear.x;
  state[1]= cmd_msg.linear.y;
  state[2]= cmd_msg.linear.z;
  state[3]= cmd_msg.angular.x*PI/180.0; // passage de ° à rad
  
  // Setup the command in articular space if the command is within available box
  if(uarm.MGIUarm(state) == 1){
    q0_command= uarm.angle[0];
    q1_command= uarm.angle[1];
    q2_command= uarm.angle[2];
    q3_command= uarm.angle[3];
    behaviour_uarm = PERFORM_COMMAND;
    fresh_command = true;   
  }
  
  // enable all isr
  sei();
}

// Gripper position callback (input position °)
void GripperCommandCallBack(const std_msgs::Int16& msg){
  // disable all isr
  cli();
  
  //copy the command
  int command = msg.data;
  
  // check if command is available
  if(command >= qGrippermax-qGrippermarg){
    command = qGrippermax-qGrippermarg;
    gripper_state = CLOSED;
  }else if(command <= qGrippermin+qGrippermarg){
    command = qGrippermin+qGrippermarg;
    gripper_state = OPEN;
  }else{
    gripper_state = UNDEFINED;
  }
  
  servGripper.write(command);
  
  // enable all isr
  sei();
}

// Front Plier order callback (input string)
void servo_cb( const std_msgs::String& cmd_msg){
  MsTimer2::stop();
  if (strcmp(cmd_msg.data, openMaxP)==0)
  {
    arretBalance = 1;
    openMaxPliers();
  }
  else if (strcmp(cmd_msg.data, openP)==0)
  {
    arretBalance = 1;
    openPliers();
  }
  else if (strcmp(cmd_msg.data, closeP)==0)
  {
    arretBalance = 1;
    closePliers();
  }
  else if (strcmp(cmd_msg.data, middle)==0)
  {
    arretBalance = 1;
    middlePliers();
  }
  else if (strcmp(cmd_msg.data, balanceP)==0)
  {
    arretBalance = 0;
    balancePliers();
  }
  MsTimer2::start();
}

// Parasol command callback (input Empty)
void ParasolCommandCallback(const std_msgs::Empty& msg){
  // disable all isr
  cli();
  
  if(parasol_state==CLOSED){
    servParasol.write(int(endSMParasol_angle));
    parasol_state=OPEN;
  }else{
    servParasol.write(int(initSMParasol_angle));
    parasol_state=OPEN;
  }
  
  // enable all isr
  sei();
}

//**********************************************************************
// Function to convert the command in Model space to servo (input in rad, output in °)
//**********************************************************************
int ConvertAngleServ0(float q0){
  int q0serv = int((q0nlincoef*q0*180.0/PI)+q0offset);
  if(q0serv > q0servmax-q0servmarg){q0serv = q0servmax-q0servmarg; }
  if(q0serv < q0servmin+q0servmarg){q0serv = q0servmin+q0servmarg;}
  return q0serv;
}

int ConvertAngleServ1(float q1){
  int q1serv = int((q1nlincoef*q1*180/PI)+q1offset);
  if(q1serv > q1servmax-q1servmarg){q1serv = q1servmax-q1servmarg;}
  if(q1serv < q1servmin+q1servmarg){q1serv = q1servmin+q1servmarg;}
  return q1serv;
}

int ConvertAngleServ2(float q2){
  int q2serv = int((-q2nlincoef*q2*180/PI)+q2offset);
  if(q2serv > q2servmax-q2servmarg){q2serv = q2servmax-q2servmarg;}
  if(q2serv < q2servmin+q2servmarg){q2serv = q2servmin+q2servmarg;}
  return q2serv;
}

int ConvertAngleServ3(float q3){
  int q3serv = int((q3nlincoef*q3*180/PI)+q3offset);
  if(q3serv > q3servmax-q3servmarg){q3serv = q3servmax-q3servmarg;}
  if(q3serv < q3servmin+q3servmarg){q3serv = q3servmin+q3servmarg;}
  return q3serv;
}

//**********************************************************************
// Rostopic definition
//**********************************************************************
//**********************************************************************
// subscriber(s)
//**********************************************************************
ros::Subscriber<geometry_msgs::Twist> UarmCommand_sub("UarmCommand", PositionCommandCallBack); //
ros::Subscriber<std_msgs::Int16> GripperCommand_sub("GripperCommand", GripperCommandCallBack);
ros::Subscriber<std_msgs::String> sub("FrontPlierCommand", servo_cb);                          //
ros::Subscriber<std_msgs::Empty> ParasolCommand_sub("ParasolCommand", ParasolCommandCallback); //

//**********************************************************************
// publisher(s)
//**********************************************************************
ros::Publisher uarmState_pub("UarmState", &current_state);      //
ros::Publisher uarmBehaviour_pub("UarmBehaviour", &behaviour);
ros::Publisher gripperState_pub("GripperState", &gripperState);
ros::Publisher ultrasonFront_pub ("UltrasonFront", &pos1); //
ros::Publisher ultrasonBack_pub ("UltrasonBack", &pos2);   //

void setup(){
  pinMode(13, OUTPUT);
  
  // init node; publisher and subscriber
  nh.getHardware()->setBaud(baudrate);
  nh.initNode();
  nh.subscribe(UarmCommand_sub);
  nh.subscribe(GripperCommand_sub);
  nh.subscribe(sub);
  nh.subscribe(ParasolCommand_sub);
  nh.advertise(uarmState_pub);
  nh.advertise(uarmBehaviour_pub);
  nh.advertise(gripperState_pub);
  nh.advertise(ultrasonFront_pub);
  nh.advertise(ultrasonBack_pub);
   
  // HW setup
  // attach pin to servo
  serv0.attach(SM0pin); 
  serv1.attach(SM1pin); 
  serv2.attach(SM2pin); 
  serv3.attach(SM3pin);
  servGripper.attach(SMGripperpin);
  servParasol.attach(SMParasolpin);
  // write init position
  serv0.write(ConvertAngleServ0(qinit[0])); //init SM0 to a custom init position
  serv1.write(ConvertAngleServ1(qinit[1])); //init SM1 to a custom init position
  serv2.write(ConvertAngleServ2(qinit[2])); //init SM2 to a custom init position
  serv3.write(ConvertAngleServ3(qinit[3])); //init SM3 to a custom init position
  servGripper.write(int(initSMGripper_angle));   //init SMGripper to a custom init position
  servParasol.write(int(initSMParasol_angle));   //init SMGParasol to a custom init position
  
  // Init the state position
  uarm.MGDUarm(qinit); // input in radian -> result in mm and rad
  X_current = uarm.state[0];
  Y_current = uarm.state[1];
  Z_current = uarm.state[2];
  theta_current = uarm.state[3]*180/PI; // passage de rad à °
  
  // Init Front Pliers
  AX12::init(1000000);
  motor[0].id=3;
  motor[1].id=4;
  motor[0].inverse = true;
  motor_init();
  
  delay(100);

  // Timer for isr setup
  MsTimer2::set(Ti, isr);
  MsTimer2::start();
}


//**********************************************************************
// Main Loop (only use for publish and read data)
//**********************************************************************
int wait_counter = 0;
float q_current[4] = {qinit[0], qinit[1], qinit[2], qinit[3]};
void loop(){
  if(wait_counter==1){ 
    // publish the Uarm position at ?Hz (10*50ms period)
    // update the current articular position
    q_current[0] = float(q0_current);
    q_current[1] = float(q1_current);
    q_current[2] = float(q2_current);
    q_current[3] = float(q3_current);
    // Calculate the state postion
    uarm.MGDUarm(q_current);
    X_current = float(uarm.state[0]);
    Y_current = float(uarm.state[1]);
    Z_current = float(uarm.state[2]);
    theta_current = float(uarm.state[3]);
    current_state.linear.x= float(X_current);             // topic in mm
    current_state.linear.y= float(Y_current);             // topic in mm
    current_state.linear.z= float(Z_current);             // topic in mm
    current_state.angular.x= float(theta_current)*180/PI; // topic in °
    // Publish the state position of the Uarm
    uarmState_pub.publish(&current_state);
  }else if(wait_counter==2){
    // Update the behaviour of the uarm
    behaviour.data = behaviour_uarm;             // cf "Uarm.h"
    // Publish the behaviour of the Uarm
    uarmBehaviour_pub.publish(&behaviour);
  }else if(wait_counter==3){
    // Update the state of the gripper
    gripperState.data = gripper_state;
    // Publish the state of the gripper
    gripperState_pub.publish(&gripperState);
  }else if(wait_counter==4){
    capt();
  }else if(wait_counter==5){  
    wait_counter=0;
  }
  
  digitalWrite(13, !digitalRead(13));
  wait_counter++;
  nh.spinOnce();
  delay(mainLoop_t);
}

//**********************************************************************
// Isr (use to update the command of the Uarm at low level)
//**********************************************************************
void isr(){
  // Case a new command is receive
  if(fresh_command==true){
    fresh_command= false; // avoid multiple handling of the same command
    boolean smallmove=((abs(q0_current-q0_command)<smvseuil*PI/180)&&(abs(q1_current-q1_command)<smvseuil*PI/180)&&(abs(q2_current-q2_command)<smvseuil*PI/180)&&(abs(q3_current-q3_command)<smvseuil*PI/180));
    // check if it is a smallmove. If it is then it will be execute AZAP because we force step_DONE == step_TODO
    if(smallmove){
      step_TODO = 0; step_DONE = 0; 
    }
    // if it isn't a smallmove, set up a procedure over time with a third degree interpolation because we force step_TODO > step_DONE
    else{
      D[0]= q0_command-q0_current; D[1]= q1_command-q1_current; D[2]= q2_command-q2_current; D[3]= q3_command-q3_current;
      qi[0] = q0_current; qi[1] = q1_current; qi[2] = q2_current; qi[3] = q3_current;
      float amplimax=max(max(max(abs(D[0]),abs(D[1])),abs(D[2])),abs(D[3]));
      tf=int((1000.0*amplimax*3)/(2*servospeed*PI/180));  // temps d'interpolation en ms
      tf=tf*coeffsec;
      step_TODO= tf/Ti;
      step_DONE= 0;
    }
  }
  
  // Case we are within an interpolation
  if(step_DONE < step_TODO){
    behaviour_uarm = PERFORM_COMMAND;
    float r=3*pow((step_DONE*float(Ti)/tf),2)-2*pow((step_DONE*float(Ti)/tf),3); //interpolation third degree
    for(int j=0;j<4;j++){
      angleGOAL[j]=qi[j]+r*D[j];
    }
    sermove(angleGOAL); 
    q0_current= angleGOAL[0]; q1_current= angleGOAL[1]; q2_current= angleGOAL[2]; q3_current= angleGOAL[3]; 
    step_DONE += 1;
  }
  
  // Case we are at the end of an interpolation or in a smallmove
  if(step_DONE == step_TODO){
    angleGOAL[0] = q0_command; angleGOAL[1] = q1_command; angleGOAL[2] = q2_command; angleGOAL[3] = q3_command; 
    sermove(angleGOAL);
    q0_current= angleGOAL[0]; q1_current= angleGOAL[1]; q2_current= angleGOAL[2]; q3_current= angleGOAL[3]; 
    step_DONE = 0; step_TODO = -1; behaviour_uarm = SLEEP_MODE;
  }
}


//**********************************************************************
// moveSM (use to move the SM in the input position in rad) 
//**********************************************************************
void sermove(float q[4]){
  serv0.write(ConvertAngleServ0(q[0])); //move SM0 to the command angle
  serv1.write(ConvertAngleServ1(q[1])); //move SM1 to the command angle
  serv2.write(ConvertAngleServ2(q[2])); //move SM2 to the command angle
  serv3.write(ConvertAngleServ3(q[3])); //move SM3 to the command angle
}

//**********************************************************************
// Front Pliers function
//**********************************************************************
void openMaxPliers()
{
  int pos = map(openMaxAngle, -150, 150, 0, 1023);
  for (int i=0; i<=1; i++)
  {
    motor[i].writeInfo (TORQUE_ENABLE, 1);
    motor[i].writeInfo (MAX_TORQUE, torqueLIM);
    motor[i].writeInfo(MOVING_SPEED, map(rotSpeed, 0, 114, 0, 1023));
    motor[i].writeInfo(GOAL_POSITION, pos);
  }
}

void openPliers()
{
  int pos = map(openAngle, -150, 150, 0, 1023);
  for (int i=0; i<=1; i++)
  {
    motor[i].writeInfo (TORQUE_ENABLE, 1);
    motor[i].writeInfo (MAX_TORQUE, torqueLIM);
    motor[i].writeInfo(MOVING_SPEED, map(rotSpeed, 0, 114, 0, 1023));
    motor[i].writeInfo(GOAL_POSITION, pos);
  }
}

void closePliers()
{
  int pos = map(closeAngle, -150, 150, 0, 1023);
  for (int i=0; i<=1; i++)
  {
    motor[i].writeInfo (TORQUE_ENABLE, 1);
    motor[i].writeInfo (MAX_TORQUE, torqueLIM);
    motor[i].writeInfo(MOVING_SPEED, map(rotSpeed, 0, 114, 0, 1023));
    motor[i].writeInfo(GOAL_POSITION, pos);
  }
}

void middlePliers()
{
  int pos = map(middleAngle, -150, 150, 0, 1023);
  for (int i=0; i<=1; i++)
  {
    motor[i].writeInfo (TORQUE_ENABLE, 1);
    motor[i].writeInfo (MAX_TORQUE, torqueLIM);
    motor[i].writeInfo(MOVING_SPEED, map(rotSpeed, 0, 114, 0, 1023));
    motor[i].writeInfo(GOAL_POSITION, pos);
  }  
}

void balancePliers()
{
  int goalPos [2];
  goalPos[0] = motor[0].readInfo (PRESENT_POSITION);
  goalPos[1] = motor[1].readInfo (PRESENT_POSITION);
  int sens = 1;
  int debattement = map(balanceAngle, -150, 150, 0, 1023);
  for (int i=0; i<=1; i++)
  {
    motor[i].writeInfo (TORQUE_ENABLE, 1);
    motor[i].writeInfo (MAX_TORQUE, torqueLIM);
  }  
  while (!arretBalance)
  {    
    for (int i=0; i<=1; i++)
    {
      goalPos[i] = goalPos[i] + sens*debattement;
      motor[i].writeInfo (GOAL_POSITION, goalPos[i]);
    }  
    sens = -sens;
  }
}
  
//Initialisation des paramètres des moteurs
void motor_init () {
 for (int i=0; i<2; i++) {
   motor[i].writeInfo (TORQUE_ENABLE, 1);
   motor[i].writeInfo (CW_COMPLIANCE_MARGIN, 2);
   motor[i].writeInfo (CCW_COMPLIANCE_MARGIN, 2);
   motor[i].writeInfo (CW_COMPLIANCE_SLOPE, 95);
   motor[i].writeInfo (CCW_COMPLIANCE_SLOPE, 95);
   motor[i].writeInfo (PUNCH, 150);
   motor[i].writeInfo (MAX_TORQUE, torqueLIM);
   motor[i].writeInfo (LIMIT_TEMPERATURE, 85);
   motor[i].writeInfo (DOWN_LIMIT_VOLTAGE, 60);
   motor[i].writeInfo (UP_LIMIT_VOLTAGE, 190);
   motor[i].writeInfo (RETURN_DELAY_TIME, 150);
   motor[i].setEndlessTurnMode(false);
 }
}

//**********************************************************************
// Ultrasonic Captor
//**********************************************************************
void capt()
{
  pos1.data=ultrasonicFront.MeasureInCentimeters();
  pos2.data=ultrasonicBack.MeasureInCentimeters();  
  
  if (pos1.data <= positionAlerte)
  {
    ultrasonFront_pub.publish(&pos1);
  } 
  if (pos2.data <= positionAlerte)
  {
    ultrasonBack_pub.publish(&pos2);
  } 
}
