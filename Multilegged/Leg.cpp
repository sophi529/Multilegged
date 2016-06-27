//
//  Leg.cpp
//  multi-legged-walker
//
//  Created by Sophie Dewil on 6/5/16.
//  Copyright © 2016 Sophie Dewil. All rights reserved.
//

#include "Leg.hpp"

#include "LeggedAgent.hpp"
// ***********************
// Methods for Leg
//
// SRMD 06/05/2017
// ***********************


#include "Random.hpp"
// load coasting circuit into old one legged body
// try in multilegged body print out time steps


// Constants

const int    LegLength = 15;
const double MaxLegForce = 0.05;
const double ForwardAngleLimit = Pi/6;
const double BackwardAngleLimit = -Pi/6;
const double MaxVelocity = 6.0;
const double MaxTorque = 0.5;
const double MaxOmega = 1.0;
const int VecSize = 5;
int j = 0;



/*
 void TLeg::Reset(double ix, double iy, int randomize, RandomState &rs)
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
// Step the insect using a general CTRNN CPG

/*
bool TLeg::ConstraintViolation()
{
    return (JointY - FootY > 20);
}
*/

// Update the leg geometry //this is one leg!!!
void TLeg::UpdateLeg(double StepSize, double vx)
{
    //cout << "Joint Y before: " << JointY << endl;
    JointY = JointY + StepSize * vx;
    //cout << "Joint Y after: " << JointY << endl;
    //cout << "Step Size: " << StepSize << endl;
    //cout << "vx: " << vx << endl;
    if (FootState == 1.0) {
        
        double angle = atan2(FootY - JointY, FootX - JointX);
        Omega = (angle - Angle)/StepSize;
        Angle = angle;
        if (Angle < BackwardAngleLimit) {Angle = BackwardAngleLimit; Omega = 0;}
        //cout << "Angle < BackwardAngleLimit" << endl;}
        if (Angle > ForwardAngleLimit) {Angle = ForwardAngleLimit; Omega = 0;}
   /**changed**/
        //if (Omega < -MaxOmega) {Omega = -MaxOmega;}
        //cout << "Omega < -MaxOmega" << endl;}
        //if (Omega > MaxOmega) {Omega = MaxOmega;}
        //FootX = JointX + LegLength * /*sin(Angle)*/cos(Angle);
        //FootY = JointY + LegLength * /*cos(Angle)*/sin(Angle);
        
    }
    else {
        //JointY = JointY + StepSize * vx;
        //vx = 0;
        Omega = Omega + StepSize * MaxTorque * (BackwardForce - ForwardForce);
        /*
        cout << " " << endl;
        cout << "Backward Force: " << BackwardForce << endl;
        cout << "Forward Force: " << ForwardForce << endl;
        cout << " " << endl;
         */
        if (Omega < -MaxOmega) {Omega = -MaxOmega;}
            //cout << "Omega < -MaxOmega" << endl;}
        if (Omega > MaxOmega) {Omega = MaxOmega;}
            //cout << "Omega > MaxOmega" << endl;}
        Angle = Angle + StepSize * Omega;
        //cout << "Angle: " << Angle << endl;
        //cout << "Omega: " << Omega << endl;
        if (Angle < BackwardAngleLimit) {Angle = BackwardAngleLimit; Omega = 0;}
            //cout << "Angle < BackwardAngleLimit" << endl;}
        if (Angle > ForwardAngleLimit) {Angle = ForwardAngleLimit; Omega = 0;}
            //cout << "Angle > ForwardAngleLimit" << endl;}
        if signbit(JointX){
            //Foot X-left
            FootX = JointX - LegLength * cos(Angle);}
        else{
            //Foot X-right
            FootX = JointX + LegLength * cos(Angle);}
        //FootX = JointX + LegLength * /*sin(Angle)*/cos(Angle);
        FootY = JointY + LegLength * /*cos(Angle)*/sin(Angle);
    }
}




/*
 void TLeg::Step2(double StepSize)
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
 Body.vx = Body.vx + StepSize * force;
 if (Body.vx < -MaxVelocity) Body.vx = -MaxVelocity;
 if (Body.vx > MaxVelocity) Body.vx = MaxVelocity;
 Body.cx = Body.cx + StepSize * Body.vx;
 // Update the leg geometry
 Leg.JointX = Leg.JointX + StepSize * Body.vx;
 if (Leg.FootState == 1.0) {
 double angle = atan2(Leg.FootX - Leg.JointX,Leg.FootY - Leg.JointY);
 Leg.Omega = (angle - Leg.Angle)/StepSize;
 Leg.Angle = angle;
 }
 else {
 Body.vx = 0.0;
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
 if (Body.cx - Leg.FootX > 20) Body.vx = 0.0;
 }
 
 
 // Step the LeggedAgent using a 1-neuron CTRNN CPG
 
 void TLeg::Step1(double StepSize)
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
 Body.vx = Body.vx + StepSize * force;
 if (Body.vx < -MaxVelocity) Body.vx = -MaxVelocity;
 if (Body.vx > MaxVelocity) Body.vx = MaxVelocity;
 Body.cx = Body.cx + StepSize * Body.vx;
 // Update the leg geometry
 Leg.JointX = Leg.JointX + StepSize * Body.vx;
 if (Leg.FootState == 1.0) {
 double angle = atan2(Leg.FootX - Leg.JointX,Leg.FootY - Leg.JointY);
 Leg.Omega = (angle - Leg.Angle)/StepSize;
 Leg.Angle = angle;
 }
 else {
 Body.vx = 0.0;
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
 if (Body.cx - Leg.FootX > 20) Body.vx = 0.0;
 }
 
 
 // Step the LeggedAgent using the optimal pattern generator
 
 void TLeg::PerfectStep(double StepSize)
 {
 double force = 0.0;
 
 // Update the leg effectors
 if (Leg.FootState == 0.0 && Leg.Angle >= ForwardAngleLimit) {Leg.FootState = 1; Leg.Omega = 0;}
 else if (Leg.FootState == 1.0 && (Body.cx - Leg.FootX > 20)) Leg.FootState = 0;
 // Compute the force applied to the body
 if (Leg.FootState == 1.0 && Leg.Angle >= BackwardAngleLimit && Leg.Angle <= ForwardAngleLimit)
 force = MaxLegForce;
 // Update the position of the body
 Body.vx = Body.vx + StepSize * force;
 if (Body.vx < -MaxVelocity) Body.vx = -MaxVelocity;
 if (Body.vx > MaxVelocity) Body.vx = MaxVelocity;
 Body.cx = Body.cx + StepSize * Body.vx;
 // Update the leg geometry
 Leg.JointX = Leg.JointX + StepSize * Body.vx;
 if (Leg.FootState == 1.0) {
 double angle = atan2(Leg.FootX - Leg.JointX,Leg.FootY - Leg.JointY);
 Leg.Omega = (angle - Leg.Angle)/StepSize;
 Leg.Angle = angle;
 }
 else {
 Body.vx = 0.0;
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
 if (Body.cx - Leg.FootX > 20) Body.vx = 0.0;
 }
 */
