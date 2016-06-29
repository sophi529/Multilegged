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
const double MaxLegForce6 = /*0.05*/ /*0.008*/0.0167;
const double MaxLegForce1 = 0.05;
const double ForwardAngleLimit = Pi/6;
const double BackwardAngleLimit = -Pi/6;
const double MaxVelocity = 6.0;
const double MaxTorque = 0.5;
const double MaxOmega = 1.0;
double StepSize = 0.1;
int inum = 5;
int neuron_num_oneleg = 5;


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
    for (int i = 0; i <=inum; i++) {
        
        cx = ix; cy = iy; vx = 0.0;
        LegVec[i].jointplacey = posjointy[i];
        LegVec[i].jointplacex = posjointx[i];
        LegVec[i].FootState = 0;
            if (randomize) LegVec[i].Angle = UniformRandom(BackwardAngleLimit,ForwardAngleLimit);
            else LegVec[i].Angle = ForwardAngleLimit;
            //else LegVec[i].Angle = 0;
        LegVec[i].Omega = LegVec[i].LegForwardForce = LegVec[i].LegBackwardForce = 0.0;
        LegVec[i].Omega = LegVec[i].BodyForwardForce = LegVec[i].BodyBackwardForce = 0.0;
        
        LegVec[i].JointY = cy + LegVec[i].jointplacey;
        LegVec[i].JointX = cx + LegVec[i].jointplacex;
        
        if signbit(LegVec[i].JointX){
            //Foot X-left
            LegVec[i].FootX = LegVec[i].JointX - LegLength * cos(LegVec[i].Angle);}
        else{
            //Foot X-right
            LegVec[i].FootX = LegVec[i].JointX + LegLength * cos(LegVec[i].Angle);}
            //Foot Y
        LegVec[i].FootY = LegVec[i].JointY + LegLength * sin(LegVec[i].Angle);
        }
        if (randomize) NervousSystem.RandomizeCircuitState(-0.1,0.1);
        else NervousSystem.RandomizeCircuitState(0.0,0.0);
}

void LeggedAgent::Reset(double ix, double iy, int randomize, RandomState &rs)
{
    for (int i = 0; i <=inum; i++) {
        
        cx = ix; cy = iy; vx = 0.0;
        LegVec[i].jointplacey = posjointy[i];
        LegVec[i].jointplacex = posjointx[i];
        LegVec[i].FootState = 0;
        if (randomize) LegVec[i].Angle = UniformRandom(BackwardAngleLimit,ForwardAngleLimit);
        else LegVec[i].Angle = ForwardAngleLimit;
        //else LegVec[i].Angle = 0;
        LegVec[i].Omega = LegVec[i].LegForwardForce = LegVec[i].LegBackwardForce = 0.0;
        LegVec[i].Omega = LegVec[i].BodyForwardForce = LegVec[i].BodyBackwardForce = 0.0;
        
        LegVec[i].JointY = cy + LegVec[i].jointplacey;
        LegVec[i].JointX = cx + LegVec[i].jointplacex;
        
        if signbit(LegVec[i].JointX){
            //Foot X-left
            LegVec[i].FootX = LegVec[i].JointX - LegLength * cos(LegVec[i].Angle);}
        else{
            //Foot X-right
            LegVec[i].FootX = LegVec[i].JointX + LegLength * cos(LegVec[i].Angle);}
        //Foot Y
        LegVec[i].FootY = LegVec[i].JointY + LegLength * sin(LegVec[i].Angle);
    }
    if (randomize) NervousSystem.RandomizeCircuitState(-0.1,0.1,rs);
    else NervousSystem.RandomizeCircuitState(0.0,0.0,rs);
}

void LeggedAgent::Step(double StepSize)
{
//    for (int i = 0; i <= inum; i++) {
        
        // Update the nervous system

    NervousSystem.EulerStep(StepSize);
    
        /*
         // Update the leg effectors
         for (int j = 13; j <= 18; j++) {
         if (Body.NervousSystem.NeuronOutput(j) > 0.5) {Body.LegVec[i].FootState = 1; Body.LegVec[i].Omega = 0;}
         else Body.LegVec[i].FootState = 0;
         
         //Body.LegVec[i].ForwardForce = Body.NSVec[i].NeuronOutput(2) * MaxLegForce;
         // Body.LegVec[i].BackwardForce = Body.NSVec[i].NeuronOutput(3) * MaxLegForce;
         }*/
//    }
    
    UpdateBodyModel(StepSize);

}

void LeggedAgent::ForceOutput()
{
    double force = 0.0;
    
    
        vector<vector<int>> doublevec;
        doublevec.resize(inum + 1);
        for (int i = 0; i <= (inum); i++) {
            doublevec[i].resize(neuron_num_oneleg);
        }

        
        int k = 1;
        for (int i = 0; i <= (inum); i++) {
            for (int j = 0; j <= neuron_num_oneleg - 1; j++) {
                doublevec[i][j] = k++;
            }
        }

    for (int i = 0; i <= inum; i++) {
       
        LegVec[i].LegForwardForce = NervousSystem.NeuronOutput(doublevec[i][1]) * MaxLegForce1;
        LegVec[i].LegBackwardForce = NervousSystem.NeuronOutput(doublevec[i][2]) * MaxLegForce1;
        LegVec[i].BodyForwardForce = NervousSystem.NeuronOutput(doublevec[i][1]) * MaxLegForce6;
        LegVec[i].BodyBackwardForce = NervousSystem.NeuronOutput(doublevec[i][2]) * MaxLegForce6;
        }


 /*
    //multi-legged walker without evolved cpg
    for (int i = 0; i <=inum; i++) {
        
        for(int k = 3; k <= 4; k++){
        //int k = 1;
        //LegVec[i].ForwardForce = NervousSystem.NeuronOutput(k++) * MaxLegForce;
            
            LegVec[i].ForwardForce = NervousSystem.NeuronOutput(2) * MaxLegForce;
            LegVec[i].BackwardForce = NervousSystem.NeuronOutput(3) * MaxLegForce;
*/
        //cout << "Foot state: " << LegVec[i].FootState << endl;
        //cout << "Neuron 1 output: " << NervousSystem.NeuronOutput(1) << endl;
    

    
    for (int i = 0; i <=inum; i++) {
    double f = LegVec[i].BodyForwardForce - LegVec[i].BodyBackwardForce;
        if (LegVec[i].FootState == 1.0)
            if ((LegVec[i].Angle >= BackwardAngleLimit && LegVec[i].Angle <= ForwardAngleLimit) ||
                (LegVec[i].Angle < BackwardAngleLimit && f < 0) ||
                (LegVec[i].Angle > ForwardAngleLimit && f > 0))
                force += f;
       
        /*
        cout << "LegVec[i].Angle: " << LegVec[i].Angle << endl;
        cout << "BackwardAngleLimit: " << BackwardAngleLimit << endl;
        cout << "ForwardAngleLimit: " << ForwardAngleLimit << endl;
        cout << "f: " << f << endl;
         
        //cout << "force: " << force << endl;
         */
    }
    NetForce = force;
    //cout << "NetForce: " << NetForce << endl;
    //cout << "force: " << force << endl;
    //cout << "netforce: " << NetForce << endl;
    /*
    LegVec[0].ForwardForce = NervousSystem.NeuronOutput(2) * MaxLegForce;
    LegVec[0].BackwardForce = NervousSystem.NeuronOutput(3) * MaxLegForce;
    double f = LegVec[0].ForwardForce - LegVec[0].BackwardForce;
    //cout << "foot state: " << LegVec[0].FootState << endl;
    //cout << "Neuron 3 output: " << NervousSystem.NeuronOutput(3) << endl;
    //cout << "FootState in force: " << LegVec[0].FootState << endl;
    if (LegVec[0].FootState == 1.0)
            if ((LegVec[0].Angle >= BackwardAngleLimit && LegVec[0].Angle <= ForwardAngleLimit) ||(LegVec[0].Angle < BackwardAngleLimit && f < 0) ||(LegVec[0].Angle > ForwardAngleLimit && f > 0))
                NetForce = f;
     */
    //cout << "NetForce set force: " << NetForce << endl;
    //cout << "angle >= backwardanglelimit and <= forwardangelimit" << (LegVec[0].Angle >= BackwardAngleLimit && LegVec[0].Angle <= ForwardAngleLimit) << endl;
    //cout << "angle < backwardanglelimit and netforce < 0" << (LegVec[0].Angle < BackwardAngleLimit && f < 0) << endl;
    //cout << "angle > forwardanglelimit and netforce > 0" << (LegVec[0].Angle > ForwardAngleLimit && f > 0) << endl;
    //force += f;
    //cout << "force: " << force << endl;
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


// Step the insect using a general CTRNN CPG

void LeggedAgent::UpdateBodyModel(double StepSize)
{
    //make a vector to hold foot states
    
    for (int i = 0; i <inum; i++) {
        if(LegVec[i].FootState == 1)
        {
            vertx.push_back(LegVec[i].FootX);
            verty.push_back(LegVec[i].FootY);
        }
    }

    //float FootVecX[6];
    //float FootVecY[6];
    
    //Six legged walker
    
        vector<vector<int>> doublevec;
        doublevec.resize(inum + 1);
        for (int i = 0; i <= (inum); i++) {
            doublevec[i].resize(neuron_num_oneleg);
        }
        
        int k = 1;
        for (int i = 0; i <= (inum); i++) {
            for (int j = 0; j <= neuron_num_oneleg - 1; j++) {
                doublevec[i][j] = k++;
            }
        }
 
    for (int i = 0; i <= inum; i++) {
        
        //for (int j = 1; j <= 3; j++) {
            if (NervousSystem.NeuronOutput(doublevec[i][0]) > 0.5) {LegVec[i].FootState = 1; LegVec[i].Omega = 0;}
            else LegVec[i].FootState = 0;
        //cout << "Neuron output" << NervousSystem.NeuronOutput(1) << endl;
        //cout << "foot state: " << LegVec[i].FootState << endl;
        //}
    }
    
    
    //cout << "Neuron output" << NervousSystem.NeuronOutput(1) << endl;

    
    //multi-legged walker with no evolved cpg
    /*
    for (int i = 0; i <= inum; i++) {
        
            if (NervousSystem.NeuronOutput(1) > 0.5) {LegVec[i].FootState = 1; LegVec[i].Omega = 0;}
            else LegVec[i].FootState = 0;
    }
     */

    ForceOutput();
    //if it violates the constraints stop it

    if (ConstraintViolation())
    {
        vx = 0.0;
        //cout << LegVec[0].JointY - LegVec[0].FootY << endl;
        
        //for (int i = 0; i <= inum; i++) {
            //LegVec[i].UpdateLeg(StepSize, vx);
        //}
    }
    // if there are less than 3 feet on the ground it falls
    /*
    else if (vertx.size() < 3)
    {vx = 0.0;
        cout << "boomsmall" << endl;}
    // if the center of gravity is outside of the feet (return 0) it falls
    else if (vertx.size() > 2 && vertx.size() < 5)
        {if (pnpoly() == 0)
            cout << "boom" << endl;
            vx = 0.0;}
     */
    // otherwise everything is chill and we can update the velocity

    

    else if (NetForce == 0)
    {vx = 0;
        if (vx < -MaxVelocity) vx = -MaxVelocity;
        if (vx > MaxVelocity) vx = MaxVelocity;}
    else
    {
            vx = vx + StepSize * NetForce;
            if (vx < -MaxVelocity) vx = -MaxVelocity;
            if (vx > MaxVelocity) vx = MaxVelocity;
        }
    //and now we update the leg based on all of that
    for (int i = 0; i <= inum; i++) {
        LegVec[i].UpdateLeg(StepSize, vx);
    }
}



//return 0 = center of gravity outside of feet
int LeggedAgent::pnpoly()
{
    
    int nvert = 6;
    double testx = 0.0;
    double testy = 0.0;
    
    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if (((verty[i]>testy) != (verty[j]>testy)) &&
            (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]))
            c = !c;
    }
    //cout << c << endl;
    return c;
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

/*
 
 void LeggedAgent::Step(double StepSize)
 {
 double force = 0.0;
 
 // Update the nervous system
 NervousSystem.EulerStep(StepSize);
 // Update the leg effectors
 if (NervousSystem.NeuronOutput(1) > 0.5) {Leg.FootState = 1; Leg.Omega = 0;}
 else Leg.FootState = 0;
 Leg.ForwardForce = NervousSystem.NeuronOutput(2) * MaxLegForce;
 Leg.BackwardForce = NervousSystem.NeuronOutput(3) * MaxLegForce;
 // Compute the force applied to the body
 // *** This is a CHANGE from the original body model that allows a supporting leg that has
 // *** passed outside of the mechanical limits to apply force in a direction that moves it
 // *** back toward that mechanical limit but not in a direction that would move it further
 // *** away.  In effect, the mechanical limits become 1-way constraints for a supporting leg.
 double f = Leg.ForwardForce - Leg.BackwardForce;
 if (Leg.FootState == 1.0)
 if ((Leg.Angle >= BackwardAngleLimit && Leg.Angle <= ForwardAngleLimit) ||
 (Leg.Angle < BackwardAngleLimit && f < 0) ||
 (Leg.Angle > ForwardAngleLimit && f > 0))
 force = f;
 // *** The original code
 //		if (Leg.FootState == 1.0 && Leg.Angle >= BackwardAngleLimit &&  Leg.Angle <= ForwardAngleLimit)
 //			force = Leg.ForwardForce - Leg.BackwardForce;
 // ***
 // Update the position of the body
 vx = vx + StepSize * force;
 if (vx < -MaxVelocity) vx = -MaxVelocity;
 if (vx > MaxVelocity) vx = MaxVelocity;
 j
 // Update the leg geometry
 Leg.JointX = Leg.JointX + StepSize * vx;
 if (Leg.FootState == 1.0) {
 double angle = atan2(Leg.FootX - Leg.JointX,Leg.FootY - Leg.JointY);
 Leg.Omega = (angle - Leg.Angle)/StepSize;
 Leg.Angle = angle;
 }
 else {
 vx = 0.0;
 Leg.Omega	= Leg.Omega + StepSize * MaxTorque * (Leg.BackwardForce - Leg.ForwardForce);
 if (Leg.Omega < -MaxOmega) Leg.Omega = -MaxOmega;
 if (Leg.Omega > MaxOmega) Leg.Omega = MaxOmega;
 Leg.Angle = Leg.Angle + StepSize * Leg.Omega;
 if (Leg.Angle < BackwardAngleLimit) {Leg.Angle = BackwardAngleLimit; Leg.Omega = 0;}
 if (Leg.Angle > ForwardAngleLimit) {Leg.Angle = ForwardAngleLimit; Leg.Omega = 0;}
 Leg.FootX = Leg.JointX + LegLength * sin(Leg.Angle);
 Leg.FootY = Leg.JointY + LegLength * cos(Leg.Angle);
 }
 // If the foot is too far back, the body becomes "unstable" and forward motion ceases
 if (cx - Leg.FootX > 20) vx = 0.0;
 }
 
 
 // Step the LeggedAgent using a 2-neuron CTRNN CPG
 
 void LeggedAgent::Step2(double StepSize)
 {
 double force = 0.0;
 
 // Update the nervous system
 NervousSystem.EulerStep(StepSize);
 // Update the leg effectors
 if (NervousSystem.NeuronOutput(1) > 0.5) {Leg.FootState = 1; Leg.Omega = 0;}
 else Leg.FootState = 0;
 Leg.ForwardForce = NervousSystem.NeuronOutput(1) * MaxLegForce;
 Leg.BackwardForce = NervousSystem.NeuronOutput(2) * MaxLegForce;
 double f = Leg.ForwardForce - Leg.BackwardForce;
 if (Leg.FootState == 1.0)
 if ((Leg.Angle >= BackwardAngleLimit && Leg.Angle <= ForwardAngleLimit) ||
 (Leg.Angle < BackwardAngleLimit && f < 0) ||
 (Leg.Angle > ForwardAngleLimit && f > 0))
 force = f;
 // Update the position of the body
 vx = vx + StepSize * force;
 if (vx < -MaxVelocity) vx = -MaxVelocity;
 if (vx > MaxVelocity) vx = MaxVelocity;
 cx = cx + StepSize * vx;
 // Update the leg geometry
 Leg.JointX = Leg.JointX + StepSize * vx;
 if (Leg.FootState == 1.0) {
 double angle = atan2(Leg.FootX - Leg.JointX,Leg.FootY - Leg.JointY);
 Leg.Omega = (angle - Leg.Angle)/StepSize;
 Leg.Angle = angle;
 }
 else {
 vx = 0.0;
 Leg.Omega	= Leg.Omega + StepSize * MaxTorque * (Leg.BackwardForce - Leg.ForwardForce);
 if (Leg.Omega < -MaxOmega) Leg.Omega = -MaxOmega;
 if (Leg.Omega > MaxOmega) Leg.Omega = MaxOmega;
 Leg.Angle = Leg.Angle + StepSize * Leg.Omega;
 if (Leg.Angle < BackwardAngleLimit) {Leg.Angle = BackwardAngleLimit; Leg.Omega = 0;}
 if (Leg.Angle > ForwardAngleLimit) {Leg.Angle = ForwardAngleLimit; Leg.Omega = 0;}
 Leg.FootX = Leg.JointX + LegLength * sin(Leg.Angle);
 Leg.FootY = Leg.JointY + LegLength * cos(Leg.Angle);
 }
 // If the foot is too far back, the body becomes "unstable" and forward motion ceases
 if (cx - Leg.FootX > 20) vx = 0.0;
 }
 
 
 // Step the LeggedAgent using a 1-neuron CTRNN CPG
 
 void LeggedAgent::Step1(double StepSize)
 {
 double force = 0.0;
 
 // Update the sensory input
 NervousSystem.SetNeuronExternalInput(1,Leg.Angle * 5.0/ForwardAngleLimit);
 // Update the nervous system
 NervousSystem.EulerStep(StepSize);
 double o = NervousSystem.NeuronOutput(1);
 // Update the leg effectors
 if (o > 0.5) {
 Leg.FootState = 1;
 Leg.Omega = 0;
 Leg.ForwardForce = 2 * (o - 0.5) * MaxLegForce;
 }
 else {
 Leg.FootState = 0;
 Leg.BackwardForce = 2 * (0.5 - o) * MaxLegForce;
 }
 // Compute the force applied to the body (*** USING THE "NEW" MODEL ***)
 double f = Leg.ForwardForce - Leg.BackwardForce;
 if (Leg.FootState == 1.0)
 if ((Leg.Angle >= BackwardAngleLimit && Leg.Angle <= ForwardAngleLimit) ||
 (Leg.Angle < BackwardAngleLimit && f < 0) ||
 (Leg.Angle > ForwardAngleLimit && f > 0))
 force = f;
 // Update the position of the body
 vx = vx + StepSize * force;
 if (vx < -MaxVelocity) vx = -MaxVelocity;
 if (vx > MaxVelocity) vx = MaxVelocity;
 cx = cx + StepSize * vx;
 // Update the leg geometry
 Leg.JointX = Leg.JointX + StepSize * vx;
 if (Leg.FootState == 1.0) {
 double angle = atan2(Leg.FootX - Leg.JointX,Leg.FootY - Leg.JointY);
 Leg.Omega = (angle - Leg.Angle)/StepSize;
 Leg.Angle = angle;
 }
 else {
 vx = 0.0;
 Leg.Omega	= Leg.Omega + StepSize * MaxTorque * (Leg.BackwardForce - Leg.ForwardForce);
 if (Leg.Omega < -MaxOmega) Leg.Omega = -MaxOmega;
 if (Leg.Omega > MaxOmega) Leg.Omega = MaxOmega;
 Leg.Angle = Leg.Angle + StepSize * Leg.Omega;
 if (Leg.Angle < BackwardAngleLimit) {Leg.Angle = BackwardAngleLimit; Leg.Omega = 0;}
 if (Leg.Angle > ForwardAngleLimit) {Leg.Angle = ForwardAngleLimit; Leg.Omega = 0;}
 Leg.FootX = Leg.JointX + LegLength * sin(Leg.Angle);
 Leg.FootY = Leg.JointY + LegLength * cos(Leg.Angle);
 }
 // If the foot is too far back, the body becomes "unstable" and forward motion ceases
 if (cx - Leg.FootX > 20) vx = 0.0;
 }
 
 
 // Step the LeggedAgent using the optimal pattern generator
 
 void LeggedAgent::PerfectStep(double StepSize)
 {
 double force = 0.0;
 
 // Update the leg effectors
 if (Leg.FootState == 0.0 && Leg.Angle >= ForwardAngleLimit) {Leg.FootState = 1; Leg.Omega = 0;}
 else if (Leg.FootState == 1.0 && (cx - Leg.FootX > 20)) Leg.FootState = 0;
 // Compute the force applied to the body
 if (Leg.FootState == 1.0 && Leg.Angle >= BackwardAngleLimit && Leg.Angle <= ForwardAngleLimit)
 force = MaxLegForce;
 // Update the position of the body
 vx = vx + StepSize * force;
 if (vx < -MaxVelocity) vx = -MaxVelocity;
 if (vx > MaxVelocity) vx = MaxVelocity;
 cx = cx + StepSize * vx;
 // Update the leg geometry
 Leg.JointX = Leg.JointX + StepSize * vx;
 if (Leg.FootState == 1.0) {
 double angle = atan2(Leg.FootX - Leg.JointX,Leg.FootY - Leg.JointY);
 Leg.Omega = (angle - Leg.Angle)/StepSize;
 Leg.Angle = angle;
 }
 else {
 vx = 0.0;
 Leg.Omega	= Leg.Omega + StepSize * MaxTorque * MaxLegForce;
 if (Leg.Omega < -MaxOmega) Leg.Omega = -MaxOmega;
 if (Leg.Omega > MaxOmega) Leg.Omega = MaxOmega;
 Leg.Angle = Leg.Angle + StepSize * Leg.Omega;
 if (Leg.Angle < BackwardAngleLimit) {Leg.Angle = BackwardAngleLimit; Leg.Omega = 0;}
 if (Leg.Angle > ForwardAngleLimit) {Leg.Angle = ForwardAngleLimit; Leg.Omega = 0;}
 Leg.FootX = Leg.JointX + LegLength * sin(Leg.Angle);
 Leg.FootY = Leg.JointY + LegLength * cos(Leg.Angle);
 }
 // If the foot is too far back, the body becomes "unstable" and forward motion ceases
 if (cx - Leg.FootX > 20) vx = 0.0;
 }
 */
