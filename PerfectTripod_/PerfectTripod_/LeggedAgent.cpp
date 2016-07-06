//
//  LeggedAgent.cpp
//  one_legged_walker
//
//  Created by Sophie Dewil on 5/31/16.
//  Copyright © 2016 Sophie Dewil. All rights reserved.
//

#include "LeggedAgent.hpp"
// ***********************
// Methods for LeggedAgent
//
// RDB 2/16/96
// ***********************


#include "Random.hpp"

// Constants

const int    LegLength = 15;
const double MaxLegForce6 = /*0.05*/ /*0.008*/0.01;
const double MaxLegForce1 = 0.5;
const double ForwardAngleLimit = Pi/6;
const double BackwardAngleLimit = -Pi/6;
const double MaxVelocity = 6.0;
const double MaxTorque = 0.5;
const double MaxOmega = 1.0;
double StepSize = 0.1;
int inum = 5;
int neuron_num_oneleg = 5;
const double Velocity_Decay = 0.999;


// *******
// Control
// *******

// Reset the state of the agent

/*
 void LeggedAgent::Reset(double ix, double iy, int randomize)
 {
 cx = ix; cy = iy; vx = 0.0;
 Leg.FootState = 0;
 if (randomize) Leg.Angle = UniformRandom(BackwardAngleLimit,ForwardAngleLimit);
 else Leg.Angle = ForwardAngleLimit;
 Leg.Omega = Leg.ForwardForce = Leg.BackwardForce = 0.0;
 Leg.JointX = cx; Leg.JointY = cy + 12.5;
 Leg.FootX = Leg.JointX + LegLength * sin(Leg.Angle);
 Leg.FootY = Leg.JointY + LegLength * cos(Leg.Angle);
 if (randomize) NervousSystem.RandomizeCircuitState(-0.1,0.1);
 else NervousSystem.RandomizeCircuitState(0.0,0.0);
 }
 */
/*
 void LeggedAgent::Reset(double ix, double iy, int randomize, RandomState &rs)
 {
 cx = ix; cy = iy; vx = 0.0;
 Leg.FootState = 0;
 if (randomize) Leg.Angle = rs.UniformRandom(BackwardAngleLimit,ForwardAngleLimit);
 else Leg.Angle = ForwardAngleLimit;
 Leg.Omega = Leg.ForwardForce = Leg.BackwardForce = 0.0;
 Leg.JointX = cx; Leg.JointY = cy + 12.5;
 Leg.FootX = Leg.JointX + LegLength * sin(Leg.Angle);
 Leg.FootY = Leg.JointY + LegLength * cos(Leg.Angle);
 if (randomize) NervousSystem.RandomizeCircuitState(-0.1,0.1,rs);
 else NervousSystem.RandomizeCircuitState(0.0,0.0,rs);
 }
 */


void LeggedAgent::Reset(double ix, double iy, int randomize)
{
    
    LegVec[0].FootState = 0;
    LegVec[4].FootState = 0;
    LegVec[2].FootState = 0;
    
    LegVec[1].FootState = 1;
    LegVec[3].FootState = 1;
    LegVec[5].FootState = 1;
    
    for (int i = 0; i <= inum; i++) {
        if(i == 0 || i == 4 || i == 2)
            LegVec[i].Angle = BackwardAngleLimit;
        else
            LegVec[i].Angle = ForwardAngleLimit;
    }
    
    for (int i = 0; i <=inum; i++) {
        
        cx = ix; cy = iy; vx = 0.0;
        LegVec[i].jointplacey = posjointy[i];
        LegVec[i].jointplacex = posjointx[i];

        LegVec[i].Omega = LegVec[i].LegForwardForce = LegVec[i].LegBackwardForce = LegVec[i].BodyForwardForce = LegVec[i].BodyBackwardForce = 0.0;
        
        LegVec[i].JointY = cy + LegVec[i].jointplacey;
        LegVec[i].JointX = cx + LegVec[i].jointplacex;
        /*
        if signbit(LegVec[i].JointX){
            //Foot X-left
            LegVec[i].FootX = LegVec[i].JointX - LegLength * cos(LegVec[i].Angle);
            LegVec[i].FootY = LegVec[i].JointY + LegLength * sin(LegVec[i].Angle);}
        else{
            //Foot X-right
            LegVec[i].FootX = LegVec[i].JointX + LegLength * cos(LegVec[i].Angle);
            LegVec[i].FootY = LegVec[i].JointY + LegLength * sin(LegVec[i].Angle);}
            //Foot Y
        */
        if (signbit(LegVec[i].JointX)){
            //Foot X-left
            LegVec[i].FootX = LegVec[i].JointX - LegLength * cos(LegVec[i].Angle);
            LegVec[i].FootY = LegVec[i].JointY + LegLength * sin(LegVec[i].Angle);}
        else{
            //Foot X-right
            LegVec[i].FootX = LegVec[i].JointX + LegLength * cos(LegVec[i].Angle);
            LegVec[i].FootY = LegVec[i].JointY + LegLength * sin(LegVec[i].Angle);}
        }
    for (int i = 0; i <= inum; i++) {
        if(i == 0 || i == 4 || i == 2)
            LegVec[i].group = 1;
        else
            LegVec[i].group = 2;
    }
    
}

void LeggedAgent::Reset(double ix, double iy, int randomize, RandomState &rs)
{
    LegVec[0].FootState = 0;
    LegVec[4].FootState = 0;
    LegVec[2].FootState = 0;
    
    LegVec[1].FootState = 1;
    LegVec[3].FootState = 1;
    LegVec[5].FootState = 1;
    
    for (int i = 0; i <= inum; i++) {
        if(i == 0 || i == 4 || i == 2)
            LegVec[i].Angle = BackwardAngleLimit;
        else
            LegVec[i].Angle = ForwardAngleLimit;
    }
    
    for (int i = 0; i <=inum; i++) {
        
        cx = ix; cy = iy; vx = 0.0;
        LegVec[i].jointplacey = posjointy[i];
        LegVec[i].jointplacex = posjointx[i];
    
        
        
        
        //if (randomize) LegVec[i].Angle = UniformRandom(BackwardAngleLimit,ForwardAngleLimit);
        //else LegVec[i].Angle = ForwardAngleLimit;
        //else LegVec[i].Angle = 0;
        LegVec[i].Omega = LegVec[i].LegForwardForce = LegVec[i].LegBackwardForce = LegVec[i].BodyForwardForce = LegVec[i].BodyBackwardForce = 0.0;
        
        LegVec[i].JointY = cy + LegVec[i].jointplacey;
        LegVec[i].JointX = cx + LegVec[i].jointplacex;
        /*
        if signbit(LegVec[i].JointX){
            //Foot X-left
            LegVec[i].FootX = LegVec[i].JointX - LegLength * cos(LegVec[i].Angle);}
        else{
            //Foot X-right
            LegVec[i].FootX = LegVec[i].JointX + LegLength * cos(LegVec[i].Angle);}
        //Foot Y
        LegVec[i].FootY = LegVec[i].JointY + LegLength * sin(LegVec[i].Angle);
         */
        if (signbit(LegVec[i].JointX)){
            //Foot X-left
            LegVec[i].FootX = LegVec[i].JointX - LegLength * cos(LegVec[i].Angle);
            LegVec[i].FootY = LegVec[i].JointY + LegLength * sin(LegVec[i].Angle);}
        else{
            //Foot X-right
            LegVec[i].FootX = LegVec[i].JointX + LegLength * cos(LegVec[i].Angle);
            LegVec[i].FootY = LegVec[i].JointY + LegLength * sin(LegVec[i].Angle);}
    }

}

void LeggedAgent::Step(double StepSize)
{


    NervousSystem.EulerStep(StepSize);

  
    UpdateBodyModel(StepSize);

}

void LeggedAgent::ForceOutput()
{
    double force = 0.0;
    
    
    
    
    for (int i = 0; i <=inum; i++) {
        if (LegVec[i].FootState == 1.0)
            if ((LegVec[i].Angle >= BackwardAngleLimit && LegVec[i].Angle <= ForwardAngleLimit))
                force += MaxLegForce6;

    }
    NetForce = force;
    
}

bool LeggedAgent::ConstraintViolation()
{
    return (LegVec[0].JointY - LegVec[0].FootY > 20)||
    (LegVec[1].JointY - LegVec[1].FootY > 20)||
    (LegVec[2].JointY - LegVec[2].FootY > 20)||
    (LegVec[3].JointY - LegVec[3].FootY > 20)||
    (LegVec[4].JointY - LegVec[4].FootY > 20)||
    (LegVec[5].JointY - LegVec[5].FootY > 20);
}




void LeggedAgent::UpdateBodyModel(double StepSize)
{
    //make a vector to hold foot states
    


    for(int i = 0; i <= inum; i++)
    {
        if(i == 5)
        {
        if (LegVec[i].FootState == 1){
            if (LegVec[0].Angle < ForwardAngleLimit) {
                LegVec[0].FootState = 0;
            }
            else if(LegVec[0].Angle >= ForwardAngleLimit)
            {
                LegVec[0].FootState = 1;
                LegVec[i].FootState = 0;
            }
                }
        else
        {
            if(LegVec[i].Angle < ForwardAngleLimit)
                LegVec[0].FootState = 1;
            else{
                LegVec[i].FootState = 1;
                LegVec[0].FootState = 0;
            }
        }
        }
        else{
        if (LegVec[i].FootState == 1){
            if (LegVec[i+1].Angle < ForwardAngleLimit) {
                LegVec[i+1].FootState = 0;
            }
            else if(LegVec[i+1].Angle >= ForwardAngleLimit)
            {
                LegVec[i+1].FootState = 1;
                LegVec[i].FootState = 0;
            }
        }
        else
        {
            if(LegVec[i].Angle < ForwardAngleLimit)
                LegVec[i+1].FootState = 1;
            else{
                LegVec[i].FootState = 1;
                LegVec[i+1].FootState = 0;
            }
        }
        }
    }
    
    
 
    ForceOutput();


    if (ConstraintViolation())
    {
        vx = 0.0;
            cout << "constraint" << endl;
        
    }
 
 else    {
        
        vx = vx * Velocity_Decay;
        if (vx < -MaxVelocity) vx = 0;
        if (vx > MaxVelocity) vx = MaxVelocity;
        
        vx = vx + StepSize * NetForce;
        if (vx < -MaxVelocity) vx = -MaxVelocity;
        if (vx > MaxVelocity) vx = MaxVelocity;
        }
    //and now we update the leg based on all of that
    for (int i = 0; i <= inum; i++) {
        LegVec[i].UpdateLeg(StepSize, vx);
    }
}




void LeggedAgent::GetFoot()
{
    for (int i = 0; i <=inum; i++) {
        cout << "FootX" << i << ": " << LegVec[i].FootX << " ";
        cout << "FootY" << i << ": " <<LegVec[i].FootY << " ";
        cout << "FootState" << i << ": " << LegVec[i].FootState << endl;
    }
}

void LeggedAgent::GetLegs()
{
    for (int i = 0; i <=inum; i++) {
        cout << "JointX" << i << ": " << LegVec[i].JointX << " ";
        cout << "JointY" << i << ": " <<LegVec[i].JointY << " " << endl;
    }
}