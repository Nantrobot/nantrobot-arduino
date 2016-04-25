/*Classe Uarm
Include calculation algorithms of direct and inverse geometric model.
Author : Alexis MARTIN && Dario PALMA && Xavier DAUPTAIN
*/

#ifndef UARM_H
#define UARM_H

//**********************************************************************
// Includes
//**********************************************************************
#include"math.h"
#include"Arduino.h"

#define SLEEP_MODE 0       // sleep mode (no new command receive),
#define PERFORM_COMMAND 1  // performing command
#define Q0_LOW 2
#define Q0_HIGH 3
#define Q1_LOW 4
#define Q1_HIGH 5
#define Q2_LOW 6
#define Q2_HIGH 7
#define Q3_LOW 8
#define Q3_HIGH 9

#define COMMAND_OK 1 

//**********************************************************************
// Uarm class definition
//**********************************************************************
class Uarm{
  private:
    const float La;  // lenght of the first arm
    const float Lb;  // lenght of the second arm
    const float Ea;  // exentricity of the arm 
    const float Eb;  // tangent exentricity of the arm
    const float Dx;  // exentricity of the robot base along x
    const float Dy;  // exentricity of the robot base along y
    const float Dz;  // exentricity of the robot base along z

  public:
    float angle[4];  // position in articular space (4 servos) express in rad
    float state[4];  // position in state space (4 DOFs) express in mm and rad
    Uarm(float la, float lb, float ea, float eb, float dx, float dy, float dz); // constructor
    void MGDUarm(float q[4]); // perform MGD of the Uarm according to the input in articular space
    int MGIUarm(float X[4]); // perform MGI of the Uarm according to the input in state space
};

#endif //UARM_H
