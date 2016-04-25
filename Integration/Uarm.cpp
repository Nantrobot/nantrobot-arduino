//**********************************************************************
// includes
//**********************************************************************
#include "Uarm.h"

//-----------------------
// Constructor
//-----------------------
Uarm::Uarm(float la, float lb, float ea, float eb, float dx, float dy, float dz): La(la), Lb(lb), Ea(ea), Eb(eb), Dx(dx), Dy(dy), Dz(dz){}

//-----------------------
// MGDUarm : perform MGD of the Uarm according to the input in articular space (input in rad)
//-----------------------
void Uarm::MGDUarm(float q[4]){
    angle[0]= q[0];  
    angle[1]= q[1];
    angle[2]= q[2];
    angle[3]= q[3];
    //---------MGD------------
    state[0]= (cos(angle[0])*(La*cos(angle[1])-Lb*cos(angle[2])+Ea))+Dx;
    state[1]= (sin(angle[0])*(La*cos(angle[1])-Lb*cos(angle[2])+Ea))+Dy;
    state[2]= La*sin(angle[1])-Lb*sin(angle[2])-Eb+Dz;
    state[3]= angle[0]+angle[3];
    //------------------------
}

//-----------------------
// MGIUarm : perform MGI of the Uarm according to the input in state space (input[0,1,2] in mm and input[3] in rad)
//-----------------------
int Uarm::MGIUarm(float X[4]){
    state[0]= X[0];
    state[1]= X[1];
    state[2]= X[2];
    state[3]= X[3];
    //---------MGI------------
    // Calcul de l'angle à la base
    angle[0] = atan2(state[1],state[0]);
    
    // passage de la base du robot comme point de référence
    float x= state[0]-Dx-Ea*cos(angle[0]); 
    float y= state[1]-Dy-Ea*sin(angle[0]);
    float z= state[2]-Dz+Eb;
    
    // Cacul des angles des 2 bras
    float i= x/cos(angle[0]);
      // -- Trick to avoid divergent singularity
    if(x < 0.2 && x > -0.2){
      i = y;
    } // -- end trick
    float theta2mtheta1= acos((La*La+Lb*Lb-i*i-z*z)/(2*La*Lb));
    float A1= La-Lb*cos(theta2mtheta1);
    float A2= -Lb*sin(theta2mtheta1);
    float s1= (A1*z-A2*i)/(A1*A1+A2*A2);
    float c1= (A1*i+A2*z)/(A1*A1+A2*A2);
    
    angle[1]= atan2(s1,c1);
    angle[2]= angle[1]+theta2mtheta1;
    
    // recopie du dernier angle 
/*    angle[3] = state[3]-angle[0];
    if(angle[3] < -76.0*PI/180.0)
        angle[3] += 2*PI;
    if(angle[3] > 89.0*PI/180.0)
        angle[3] -= 2*PI;*/
    angle[3] = state[3];

    // sécurité de commande 
/*    if(angle[0] < 5.0*PI/180.0){
      return Q0_LOW;
    }else if(angle[0] > 178.0*PI/180.0){
      return Q0_HIGH;
    }else if(angle[1] < 7.0*PI/180.0){
      return Q1_LOW;
    }else if(angle[1] > 115.0*PI/180.0){
      return Q1_HIGH;
    }else if(angle[2] < 72.0*PI/180.0){
      return Q2_LOW;
    }else if(angle[2] > 182.0*PI/180.0){
      return Q2_HIGH;
    }else if(angle[3] < -76.0*PI/180.0){
      return Q3_LOW;
    }else if(angle[3] > 89.0*PI/180.0){
      return Q3_HIGH;
    }else{*/
      return COMMAND_OK;
   // }
    //------------------------
}
