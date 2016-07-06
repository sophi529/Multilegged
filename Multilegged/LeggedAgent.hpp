//
//  LeggedAgent.h
//  one_legged_walker
//
//  Created by Sophie Dewil on 5/31/16.
//  Copyright © 2016 Sophie Dewil. All rights reserved.
//

#ifndef LeggedAgent_h
#define LeggedAgent_h
// *************************
// A class for legged agents
//
// RDB 2/16/96
// *************************

#pragma once

#include "CTRNN.h"
#include "Leg.hpp"
#include <vector>

// Global constants

const double Pi = 3.1415926;


// The Leg class declaration
/**
 class TLeg {
 public:
 // The constructor
 TLeg() {};
 // The destructor
 ~TLeg() {};
 
 // Accessors
 
 double Angle, Omega, ForwardForce, BackwardForce;
 double FootX, FootY, JointX, JointY;
 double FootState;
 };
 */

// The LeggedAgent class declaration

class LeggedAgent {
public:
    // The constructor
    LeggedAgent(double ix = 0.0, double iy = 0.0)
    {
        Reset(ix,iy);
    };
    // The destructor
    ~LeggedAgent() {};
    
    // Accessors
    double PositionX(void) {return cx;};
    void SetPositionX(double newx) {cx = newx;};
    double NetForce;
    
    // Control
    void Reset(double ix, double iy, int randomize = 0);
    void Reset(double ix, double iy, int randomize, RandomState &rs);
    void Step(double StepSize);
    void Step2(double StepSize);
    void Step1(double StepSize);
    void PerfectStep(double StepSize);
    void UpdateBodyModel(double StepSize);
    void ForceOutput();
    bool ConstraintViolation();
    void GetFoot();
    void GetLegs();
    int pnpoly();
    int balance();
    
    double cx, cy, vx;
    
    CTRNN NervousSystem;
    
    TLeg Leg1; //(-8, 15, NervousSystem1);
    TLeg Leg2; //(8, 15, NervousSystem2);
    TLeg Leg3; //(-12, 0, NervousSystem3);
    TLeg Leg4; //(12, 0, NervousSystem4);
    TLeg Leg5; //(-9, -23, NervousSystem5);
    TLeg Leg6; //(9, -23, NervousSystem6);
    
    //TVector<TLeg> LegVec;
    
    
    //make positions
    /*
    double pos1x = -8;
    double pos2x = 8;
    double pos3x = -12;
    double pos4x = 12;
    double pos5x = -9;
    double pos6x = 9;
    */
    
    double pos1x = -8;
    double pos2x = 8;
    double pos3x = 12;
    double pos4x = 9;
    double pos5x = -9;
    double pos6x = -12;
    
    /*
    double pos1y = 15;
    double pos2y = 15;
    double pos3y = 0;
    double pos4y = 0;
    double pos5y = -23;
    double pos6y = -23;
     */
    
    double pos1y = 16;
    double pos2y = 16;
    double pos3y = 0;
    double pos4y = -23;
    double pos5y = -23;
    double pos6y = 0;
    
    /*
     TVector<double> posjoint;
     
     TVector<double> posfootx;
     
     TVector<double> posfooty;
     
     void makePosJoint()
     {
     posjoint[0] = pos1y;
     posjoint[1] = pos2y;
     posjoint[2] = pos3y;
     posjoint[3] = pos4y;
     posjoint[4] = pos5y;
     posjoint[5] = pos6y;
     }
     
     
     void MakeLegVec()
     {
     LegVec[0] = Leg1;
     LegVec[1] = Leg2;
     LegVec[2] = Leg3;
     LegVec[3] = Leg4;
     LegVec[4] = Leg5;
     LegVec[5] = Leg6;
     }
     */
    TLeg LegVec[6] = {Leg1, Leg2, Leg3, Leg4, Leg5, Leg6};
    //TLeg LegVec[2] = {Leg3, Leg4};
    
    double posjointy[6] = {pos1y, pos2y, pos3y, pos4y, pos5y, pos6y};
    //double posjointy[2] = {pos3y, pos4y};
    
    double posjointx[6] = {pos1x, pos2x, pos3x, pos4x, pos5x, pos6x};
    //double posjointx[2] = {pos3x, pos4x};
    std::vector<double> vertx;
    std::vector<double> verty;

};

#endif /* LeggedAgent_h */
