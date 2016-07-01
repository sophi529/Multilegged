//
//  Leg.hpp
//  multi-legged-walker
//
//  Created by Sophie Dewil on 6/5/16.
//  Copyright © 2016 Sophie Dewil. All rights reserved.
//

#ifndef Leg_hpp
#define Leg_hpp

#include <stdio.h>
#include "CTRNN.h"

class TLeg {
public:
    // The constructor
    TLeg() /*(int side, double location, CTRNN NervousSystem)*/ {};
    // The destructor
    ~TLeg() {};
    
    // Accessors
    
    double Angle, Omega, LegForwardForce, LegBackwardForce, BodyForwardForce, BodyBackwardForce;
    double FootX, FootY, JointX, JointY;
    double jointplacex, jointplacey;
    double FootState;
    
    
    //Positions
    
    
    
    //double FootPositionX(void) {return fx;};
    //void SetFootPositionX(double newx) {fx = newx;};
    
    // Control
    //void Reset(double ix, double iy, int randomize = 0);
    //void Reset(double ix, double iy, int randomize, RandomState &rs);
    
    
    // Control
    
    void Step(double StepSize);
    void Step2(double StepSize);
    void Step1(double StepSize);
    void PerfectStep(double StepSize);
    void CheckConstraints();
    void BodyForce();
    void UpdateLeg(double StepSize, double velocity);
    void Reset(double ix, double iy, int randomize = 0);
    void Reset(double ix, double iy, int randomize, RandomState &rs);
    bool ConstraintViolation();

    
    
    // double cx, cy, vx;
    
    
};
//Initial Nervous Systems


//CTRNN NervousSystem;
/*
 CTRNN NervousSystem1;
 CTRNN NervousSystem2;
 CTRNN NervousSystem3;
 CTRNN NervousSystem4;
 CTRNN NervousSystem5;
 CTRNN NervousSystem6;
 
 
 
 
 //initialize legs
 
 //TLeg Leg (-8, 15, NervousSystem);
 TLeg Leg1 (-8, 15, NervousSystem1);
 TLeg Leg2 (8, 15, NervousSystem2);
 TLeg Leg3 (-12, 0, NervousSystem3);
 TLeg Leg4 (12, 0, NervousSystem4);
 TLeg Leg5 (-9, -23, NervousSystem5);
 TLeg Leg6 (9, -23, NervousSystem6);
 
 //TVector<TLeg> LegVec;
 
 //TVector<CTRNN> NSVec;
 */
//TLeg Leg;

#endif /* Leg_hpp */

