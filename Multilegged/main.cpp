//


/************************************************************/

/************************************************************/
//  main.cpp
//  one_legged_walker
//
//  Created by Sophie Dewil on 5/31/16.
//  Copyright © 2016 Sophie Dewil. All rights reserved.
//

//in swing phase set vx to 0

#include <iostream>

// *************************************************************
// main.cpp
//
// Run a given CTRNN walker
//
// 8/27/94	Created
// 5/25/02  Revised for OS X
// *************************************************************
//decrease max leg force
//cut down max velocity
//velocity decay
//coevolve body with nervous system
//dont worry about other connections

#include "LeggedAgent.hpp"
#include "Leg.hpp"
#include "CTRNN.h"
#include "Random.hpp"
//#include <iostream>
#include <fstream>
#include "VectorMatrix.h"
#include "Tsearch.hpp"
#include <cmath>

// Global constants
const double StepSize = 0.1;
const double RunDuration = 250;
const long RandomSeed = 1;
//no connections evolved
const bool Model1 = false;
//all six connections evolved
const bool Model2 = false;
//contralateral connections evolved
const bool Model3 = false;
//ipsilateral connections evolved
const bool Model4 = true;



//CHANGED

int neuron_num = /*30*/42;
int onelegneuron_num = /*5*/7;

double vector_fill = 2;


/*double*/int vector_size = /*112*/ /*84*/ 91;
int vector_size_Model1 = 77 /*91*/;
double min_bias = -10;
double max_bias = 10;

double min_time = 0.5;
double max_time = 10;

double min_weight = -10;
double max_weight = 10;




 void Assign_params(TVector<double> &v, CTRNN &NervousSystem)
 {
     
     //initialize leg helpers
     int leg1[onelegneuron_num];
     int leg2[onelegneuron_num];
     int leg3[onelegneuron_num];
     int leg4[onelegneuron_num];
     int leg5[onelegneuron_num];
     int leg6[onelegneuron_num];
     
     int t = 1;
     for(int i = 0; i <= onelegneuron_num-1; i++)
         leg1[i] = t++;
     for(int i = 0; i <= onelegneuron_num-1; i++)
         leg2[i] = t++;
     for(int i = 0; i <= onelegneuron_num-1; i++)
         leg3[i] = t++;
     for(int i = 0; i <= onelegneuron_num-1; i++)
         leg4[i] = t++;
     for(int i = 0; i <= onelegneuron_num-1; i++)
         leg5[i] = t++;
     for(int i = 0; i <= onelegneuron_num-1; i++)
         leg6[i] = t++;
     
     int vi = 1;
     int vim = 1;

  if (Model1)
      
  {
 for(int i = 1;i <= onelegneuron_num;i++)
  {NervousSystem.SetNeuronBias(i, MapSearchParameter(v[vim++], min_bias, max_bias));}

     for(int i = 1;i <= onelegneuron_num;i++) {
         double tau = MapSearchParameter(v[vim++], min_time, max_time);
         NervousSystem.SetNeuronTimeConstant(i, tau);
     }
     
      //copy this to all 6 legs
      for(int i = 0;i <= onelegneuron_num - 1;i++)
        {NervousSystem.SetNeuronBias(leg2[i], NervousSystem.NeuronBias(leg1[i]));
          NervousSystem.SetNeuronBias(leg3[i], NervousSystem.NeuronBias(leg1[i]));
          NervousSystem.SetNeuronBias(leg4[i], NervousSystem.NeuronBias(leg1[i]));
          NervousSystem.SetNeuronBias(leg5[i], NervousSystem.NeuronBias(leg1[i]));
            NervousSystem.SetNeuronBias(leg6[i], NervousSystem.NeuronBias(leg1[i]));
        }
      for(int i = 0;i <= onelegneuron_num - 1;i++) {
          NervousSystem.SetNeuronTimeConstant(leg2[i], NervousSystem.NeuronTimeConstant(leg1[i]));
          NervousSystem.SetNeuronTimeConstant(leg3[i], NervousSystem.NeuronTimeConstant(leg1[i]));
          NervousSystem.SetNeuronTimeConstant(leg4[i], NervousSystem.NeuronTimeConstant(leg1[i]));
          NervousSystem.SetNeuronTimeConstant(leg5[i], NervousSystem.NeuronTimeConstant(leg1[i]));
          NervousSystem.SetNeuronTimeConstant(leg6[i], NervousSystem.NeuronTimeConstant(leg1[i]));
      }

  }
/*
     
  else
  {
 for(int i = 1;i <= neuron_num;i++)
 NervousSystem.SetNeuronBias(i, MapSearchParameter(v[vi++], min_bias, max_bias));
 
 for(int i = 1;i <= neuron_num;i++) {
 double tau = MapSearchParameter(v[vi++], min_time, max_time);
 NervousSystem.SetNeuronTimeConstant(i, tau);
 }
//for(int i = 1; i <= neuron_num; i++){
 //   NervousSystem.SetNeuronGain(i, MapSearchParameter(v[vi++], min_bias, max_bias));
//}
  }
*/
     
     else
     {
         for(int i = 1;i <= onelegneuron_num;i++)
         {NervousSystem.SetNeuronBias(i, MapSearchParameter(v[vi++], min_bias, max_bias));}
         
         for(int i = 1;i <= onelegneuron_num;i++) {
             double tau = MapSearchParameter(v[vi++], min_time, max_time);
             NervousSystem.SetNeuronTimeConstant(i, tau);
         }
         
         //copy this to all 6 legs
         for(int i = 0;i <= onelegneuron_num - 1;i++)
         {NervousSystem.SetNeuronBias(leg2[i], NervousSystem.NeuronBias(leg1[i]));
             NervousSystem.SetNeuronBias(leg3[i], NervousSystem.NeuronBias(leg1[i]));
             NervousSystem.SetNeuronBias(leg4[i], NervousSystem.NeuronBias(leg1[i]));
             NervousSystem.SetNeuronBias(leg5[i], NervousSystem.NeuronBias(leg1[i]));
             NervousSystem.SetNeuronBias(leg6[i], NervousSystem.NeuronBias(leg1[i]));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++) {
             NervousSystem.SetNeuronTimeConstant(leg2[i], NervousSystem.NeuronTimeConstant(leg1[i]));
             NervousSystem.SetNeuronTimeConstant(leg3[i], NervousSystem.NeuronTimeConstant(leg1[i]));
             NervousSystem.SetNeuronTimeConstant(leg4[i], NervousSystem.NeuronTimeConstant(leg1[i]));
             NervousSystem.SetNeuronTimeConstant(leg5[i], NervousSystem.NeuronTimeConstant(leg1[i]));
             NervousSystem.SetNeuronTimeConstant(leg6[i], NervousSystem.NeuronTimeConstant(leg1[i]));
         }
         
     }
      
      
     
     
     /*
     if(!Model1)
     {
     //interconnections in one leg (What i was using to test things)
     for(int i = 0;i <= (onelegneuron_num - 1);i++){
         for(int j = 0; j <= (onelegneuron_num - 1); j++){
             NervousSystem.SetConnectionWeight(leg1[i], leg1[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg2[i], leg2[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg3[i], leg3[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg4[i], leg4[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg5[i], leg5[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg6[i], leg6[j], MapSearchParameter(v[vi++], min_weight, max_weight));
         }}
     }*/
     if(!Model1)
     {
         
         //copy one set of connection weights for each leg
         for(int i = 0;i <= (onelegneuron_num - 1);i++){
             for(int j = 0; j <= (onelegneuron_num - 1); j++){
                 NervousSystem.SetConnectionWeight(leg1[i], leg1[j], MapSearchParameter(v[vi++], min_weight, max_weight));
                 
                 NervousSystem.SetConnectionWeight(leg2[i], leg2[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
                 NervousSystem.SetConnectionWeight(leg3[i], leg3[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
                 NervousSystem.SetConnectionWeight(leg4[i], leg4[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
                 NervousSystem.SetConnectionWeight(leg5[i], leg5[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
                 NervousSystem.SetConnectionWeight(leg6[i], leg6[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
             }
         }
     }
     
     else
     {
      
     //copy one set of connection weights for each leg
     for(int i = 0;i <= (onelegneuron_num - 1);i++){
         for(int j = 0; j <= (onelegneuron_num - 1); j++){
             NervousSystem.SetConnectionWeight(leg1[i], leg1[j], MapSearchParameter(v[vim++], min_weight, max_weight));
             
             NervousSystem.SetConnectionWeight(leg2[i], leg2[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
             NervousSystem.SetConnectionWeight(leg3[i], leg3[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
             NervousSystem.SetConnectionWeight(leg4[i], leg4[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
             NervousSystem.SetConnectionWeight(leg5[i], leg5[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
             NervousSystem.SetConnectionWeight(leg6[i], leg6[j], NervousSystem.ConnectionWeight(leg1[i], leg1[j]));
         }
     }
     }
     
     
     
     

     
     //test connections 1
     
     if (Model1) {
         
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg1[i], leg2[i], MapSearchParameter(v[vim++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg1[i], leg6[i], MapSearchParameter(v[vim++], min_weight, max_weight));
         }
         
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg2[i], leg1[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             NervousSystem.SetConnectionWeight(leg6[i], leg1[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg6[i], leg3[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             NervousSystem.SetConnectionWeight(leg3[i], leg6[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             NervousSystem.SetConnectionWeight(leg5[i], leg4[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             NervousSystem.SetConnectionWeight(leg4[i], leg5[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             NervousSystem.SetConnectionWeight(leg6[i], leg5[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg5[i], leg6[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg2[i], leg3[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg3[i], leg2[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg3[i], leg4[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg4[i], leg3[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
          
         }
        
         //return;

     }
      /*
     //Test connections 2
     if(Model1) {
         for (int i = 0; i <= onelegneuron_num-1; i++) {
          
             NervousSystem.SetConnectionWeight(leg1[i], leg2[i], MapSearchParameter(v[vim++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg2[i], leg1[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             NervousSystem.SetConnectionWeight(leg4[i], leg5[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             NervousSystem.SetConnectionWeight(leg5[i], leg4[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             
             NervousSystem.SetConnectionWeight(leg6[i], leg3[i], MapSearchParameter(v[vim++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg3[i], leg6[i], NervousSystem.ConnectionWeight(leg6[i], leg3[i]));
             
             NervousSystem.SetConnectionWeight(leg6[i], leg1[i], MapSearchParameter(v[vim++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg6[i], leg5[i], NervousSystem.ConnectionWeight(leg6[i], leg1[i]));
             NervousSystem.SetConnectionWeight(leg3[i], leg2[i], NervousSystem.ConnectionWeight(leg6[i], leg1[i]));
             NervousSystem.SetConnectionWeight(leg3[i], leg4[i], NervousSystem.ConnectionWeight(leg6[i], leg1[i]));
             
             NervousSystem.SetConnectionWeight(leg1[i], leg6[i], MapSearchParameter(v[vim++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg5[i], leg6[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg2[i], leg3[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg4[i], leg3[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
         }
     }
      */
     
     //the legs are completely interconnected (contra and ipsalaterally)
     if(Model2) {
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg1[i], leg2[i], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg1[i], leg6[i], MapSearchParameter(v[vi++], min_weight, max_weight));
             
             NervousSystem.SetConnectionWeight(leg2[i], leg1[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             NervousSystem.SetConnectionWeight(leg2[i], leg3[i], MapSearchParameter(v[vi++], min_weight, max_weight));

             NervousSystem.SetConnectionWeight(leg3[i], leg2[i], NervousSystem.ConnectionWeight(leg2[i], leg3[i]));
             NervousSystem.SetConnectionWeight(leg3[i], leg4[i], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg3[i], leg6[i], MapSearchParameter(v[vi++], min_weight, max_weight));

             NervousSystem.SetConnectionWeight(leg4[i], leg3[i], NervousSystem.ConnectionWeight(leg3[i], leg4[i]));
             NervousSystem.SetConnectionWeight(leg4[i], leg5[i], MapSearchParameter(v[vi++], min_weight, max_weight));

             NervousSystem.SetConnectionWeight(leg5[i], leg4[i], NervousSystem.ConnectionWeight(leg4[i], leg5[i]));
             NervousSystem.SetConnectionWeight(leg5[i], leg6[i], MapSearchParameter(v[vi++], min_weight, max_weight));

             NervousSystem.SetConnectionWeight(leg6[i], leg1[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg6[i], leg3[i], NervousSystem.ConnectionWeight(leg3[i], leg6[i]));
             NervousSystem.SetConnectionWeight(leg6[i], leg5[i], NervousSystem.ConnectionWeight(leg5[i], leg6[i]));
         }
     }
     
     //the legs are contralaterally connected
     if(Model3){
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg1[i], leg2[i], MapSearchParameter(v[vi++], min_weight, max_weight));
     }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg2[i], leg1[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg6[i], leg3[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg3[i], leg6[i], NervousSystem.ConnectionWeight(leg6[i], leg3[i]));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg5[i], leg4[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg4[i], leg5[i], NervousSystem.ConnectionWeight(leg5[i], leg4[i]));
         }
         NervousSystem.SetConnectionWeight(leg1[0], leg6[0], -6.4634477);
         NervousSystem.SetConnectionWeight(leg1[1], leg6[1], -5.4783309);
         NervousSystem.SetConnectionWeight(leg1[2], leg6[2], 6.38807278);
         NervousSystem.SetConnectionWeight(leg1[3], leg6[3], -5.3927392);
         NervousSystem.SetConnectionWeight(leg1[4], leg6[4], 7.97967587);
         NervousSystem.SetConnectionWeight(leg1[5], leg6[5], -1.6330458);
         NervousSystem.SetConnectionWeight(leg1[6], leg6[6], -7.224905);
         
         NervousSystem.SetConnectionWeight(leg6[0], leg1[0], -6.4634477);
         NervousSystem.SetConnectionWeight(leg6[1], leg1[1], -5.4783309);
         NervousSystem.SetConnectionWeight(leg6[2], leg1[2], 6.38807278);
         NervousSystem.SetConnectionWeight(leg6[3], leg1[3], -5.3927392);
         NervousSystem.SetConnectionWeight(leg6[4], leg1[4], 7.97967587);
         NervousSystem.SetConnectionWeight(leg6[5], leg1[5], -1.6330458);
         NervousSystem.SetConnectionWeight(leg6[6], leg1[6], -7.224905);
         
         NervousSystem.SetConnectionWeight(leg5[0], leg6[0], -6.4634477);
         NervousSystem.SetConnectionWeight(leg5[1], leg6[1], -5.4783309);
         NervousSystem.SetConnectionWeight(leg5[2], leg6[2], 6.38807278);
         NervousSystem.SetConnectionWeight(leg5[3], leg6[3], -5.3927392);
         NervousSystem.SetConnectionWeight(leg5[4], leg6[4], 7.97967587);
         NervousSystem.SetConnectionWeight(leg5[5], leg6[5], -1.6330458);
         NervousSystem.SetConnectionWeight(leg5[6], leg6[6], -7.224905);
         
         NervousSystem.SetConnectionWeight(leg6[0], leg5[0], -6.4634477);
         NervousSystem.SetConnectionWeight(leg6[1], leg5[1], -5.4783309);
         NervousSystem.SetConnectionWeight(leg6[2], leg5[2], 6.38807278);
         NervousSystem.SetConnectionWeight(leg6[3], leg5[3], -5.3927392);
         NervousSystem.SetConnectionWeight(leg6[4], leg5[4], 7.97967587);
         NervousSystem.SetConnectionWeight(leg6[5], leg5[5], -1.6330458);
         NervousSystem.SetConnectionWeight(leg6[6], leg5[6], -7.224905);
         
         NervousSystem.SetConnectionWeight(leg2[0], leg3[0], -6.4634477);
         NervousSystem.SetConnectionWeight(leg2[1], leg3[1], -5.4783309);
         NervousSystem.SetConnectionWeight(leg2[2], leg3[2], 6.38807278);
         NervousSystem.SetConnectionWeight(leg2[3], leg3[3], -5.3927392);
         NervousSystem.SetConnectionWeight(leg2[4], leg3[4], 7.97967587);
         NervousSystem.SetConnectionWeight(leg2[5], leg3[5], -1.6330458);
         NervousSystem.SetConnectionWeight(leg2[6], leg3[6], -7.224905);
         
         NervousSystem.SetConnectionWeight(leg3[0], leg2[0], -6.4634477);
         NervousSystem.SetConnectionWeight(leg3[1], leg2[1], -5.4783309);
         NervousSystem.SetConnectionWeight(leg3[2], leg2[2], 6.38807278);
         NervousSystem.SetConnectionWeight(leg3[3], leg2[3], -5.3927392);
         NervousSystem.SetConnectionWeight(leg3[4], leg2[4], 7.97967587);
         NervousSystem.SetConnectionWeight(leg3[5], leg2[5], -1.6330458);
         NervousSystem.SetConnectionWeight(leg3[6], leg2[6], -7.224905);
         
         NervousSystem.SetConnectionWeight(leg3[0], leg4[0], -6.4634477);
         NervousSystem.SetConnectionWeight(leg3[1], leg4[1], -5.4783309);
         NervousSystem.SetConnectionWeight(leg3[2], leg4[2], 6.38807278);
         NervousSystem.SetConnectionWeight(leg3[3], leg4[3], -5.3927392);
         NervousSystem.SetConnectionWeight(leg3[4], leg4[4], 7.97967587);
         NervousSystem.SetConnectionWeight(leg3[5], leg4[5], -1.6330458);
         NervousSystem.SetConnectionWeight(leg3[6], leg4[6], -7.224905);
         
         NervousSystem.SetConnectionWeight(leg4[0], leg3[0], -6.4634477);
         NervousSystem.SetConnectionWeight(leg4[1], leg3[1], -5.4783309);
         NervousSystem.SetConnectionWeight(leg4[2], leg3[2], 6.38807278);
         NervousSystem.SetConnectionWeight(leg4[3], leg3[3], -5.3927392);
         NervousSystem.SetConnectionWeight(leg4[4], leg3[4], 7.97967587);
         NervousSystem.SetConnectionWeight(leg4[5], leg3[5], -1.6330458);
         NervousSystem.SetConnectionWeight(leg4[6], leg3[6], -7.224905);
     }
     
     //the legs are ipsilaterally connected
     if(Model4){
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg1[i], leg6[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg6[i], leg1[i], NervousSystem.ConnectionWeight(leg1[i], leg6[i]));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg6[i], leg5[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg5[i], leg6[i], NervousSystem.ConnectionWeight(leg6[i], leg5[i]));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg2[i], leg3[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg3[i], leg2[i], NervousSystem.ConnectionWeight(leg2[i], leg3[i]));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg3[i], leg4[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg4[i], leg3[i], NervousSystem.ConnectionWeight(leg3[i], leg4[i]));
         }
         NervousSystem.SetConnectionWeight(leg1[0], leg2[0], -0.5385094);
         NervousSystem.SetConnectionWeight(leg1[0], leg2[0], -9.4061168);
         NervousSystem.SetConnectionWeight(leg1[0], leg2[0], 0.94624972);
         NervousSystem.SetConnectionWeight(leg1[0], leg2[0], -9.93544);
         NervousSystem.SetConnectionWeight(leg1[0], leg2[0], 4.73082539);
         NervousSystem.SetConnectionWeight(leg1[0], leg2[0], -8.1872014);
         NervousSystem.SetConnectionWeight(leg1[0], leg2[0], 3.1424716);
         
         NervousSystem.SetConnectionWeight(leg2[0], leg1[0], -0.5385094);
         NervousSystem.SetConnectionWeight(leg2[0], leg1[0], -9.4061168);
         NervousSystem.SetConnectionWeight(leg2[0], leg1[0], 0.94624972);
         NervousSystem.SetConnectionWeight(leg2[0], leg1[0], -9.93544);
         NervousSystem.SetConnectionWeight(leg2[0], leg1[0], 4.73082539);
         NervousSystem.SetConnectionWeight(leg2[0], leg1[0], -8.1872014);
         NervousSystem.SetConnectionWeight(leg2[0], leg1[0], 3.1424716);
         
         NervousSystem.SetConnectionWeight(leg6[0], leg3[0], -0.5385094);
         NervousSystem.SetConnectionWeight(leg6[0], leg3[0], -9.4061168);
         NervousSystem.SetConnectionWeight(leg6[0], leg3[0], 0.94624972);
         NervousSystem.SetConnectionWeight(leg6[0], leg3[0], -9.93544);
         NervousSystem.SetConnectionWeight(leg6[0], leg3[0], 4.73082539);
         NervousSystem.SetConnectionWeight(leg6[0], leg3[0], -8.1872014);
         NervousSystem.SetConnectionWeight(leg6[0], leg3[0], 3.1424716);
         
         NervousSystem.SetConnectionWeight(leg3[0], leg6[0], -0.5385094);
         NervousSystem.SetConnectionWeight(leg3[0], leg6[0], -9.4061168);
         NervousSystem.SetConnectionWeight(leg3[0], leg6[0], 0.94624972);
         NervousSystem.SetConnectionWeight(leg3[0], leg6[0], -9.93544);
         NervousSystem.SetConnectionWeight(leg3[0], leg6[0], 4.73082539);
         NervousSystem.SetConnectionWeight(leg3[0], leg6[0], -8.1872014);
         NervousSystem.SetConnectionWeight(leg3[0], leg6[0], 3.1424716);
         
         NervousSystem.SetConnectionWeight(leg5[0], leg4[0], -0.5385094);
         NervousSystem.SetConnectionWeight(leg5[0], leg4[0], -9.4061168);
         NervousSystem.SetConnectionWeight(leg5[0], leg4[0], 0.94624972);
         NervousSystem.SetConnectionWeight(leg5[0], leg4[0], -9.93544);
         NervousSystem.SetConnectionWeight(leg5[0], leg4[0], 4.73082539);
         NervousSystem.SetConnectionWeight(leg5[0], leg4[0], -8.1872014);
         NervousSystem.SetConnectionWeight(leg5[0], leg4[0], 3.1424716);
         
         NervousSystem.SetConnectionWeight(leg4[0], leg5[0], -0.5385094);
         NervousSystem.SetConnectionWeight(leg4[0], leg5[0], -9.4061168);
         NervousSystem.SetConnectionWeight(leg4[0], leg5[0], 0.94624972);
         NervousSystem.SetConnectionWeight(leg4[0], leg5[0], -9.93544);
         NervousSystem.SetConnectionWeight(leg4[0], leg5[0], 4.73082539);
         NervousSystem.SetConnectionWeight(leg4[0], leg5[0], -8.1872014);
         NervousSystem.SetConnectionWeight(leg4[0], leg5[0], 3.1424716);

     }
 }

 
   /*
 int leg1[onelegneuron_num];
     int t = 1;
     for(int i = 0; i <= onelegneuron_num-1; i++)
         leg1[i] = t++;
     
     int vim = 1;
     for(int i = 1;i <= onelegneuron_num;i++)
        {NervousSystem.SetNeuronBias(i, MapSearchParameter(v[vim++], min_bias, max_bias));}
         
     for(int i = 1;i <= onelegneuron_num;i++) {
         double tau = MapSearchParameter(v[vim++], min_time, max_time);
         NervousSystem.SetNeuronTimeConstant(i, tau);
         }
     for(int i = 0;i <= (onelegneuron_num - 1);i++){
         for(int j = 0; j <= (onelegneuron_num - 1); j++){
             NervousSystem.SetConnectionWeight(leg1[i], leg1[j], MapSearchParameter(v[vim++], min_weight, max_weight));
         }
     }
 }
*/
void DumpCircuit(int, TVector<double> &v)
{
    CTRNN c(neuron_num);
    Assign_params(v,c);
    
    ofstream f("/Users/sophi529/Desktop/best.ns");
    f << c;

}

double evaluate(TVector<double> &v, RandomState &r){

    LeggedAgent Insect;
    
    Insect.NervousSystem.SetCircuitSize(neuron_num);
    
    Assign_params(v, Insect.NervousSystem);
    
    Insect.NervousSystem.RandomizeCircuitState(0,0.1,r);
   

    Insect.Reset(0, 0, 1, r);
 
    for (double time = 0; time < RunDuration; time += StepSize) {
        
        Insect.Step(StepSize);
       
    }
    
//if fitness is negative return 0

    if (Insect.LegVec[2].JointY/RunDuration <= 0)
        return 0;
    else
        return Insect.LegVec[2].JointY/RunDuration;

    
}


int main(int argc, char* argv[])
//int main(int seed)
{
   
    int seed = atoi(argv[1]);
    string walk = argv[2];
    string info = argv[3];
    string help = argv[4];
  
    //int seed = 16263646;
    //string walk = "/Users/sophi529/Desktop/output/walk_186.dat";
    //string info = "/Users/sophi529/Desktop/output/186.dat";
    //string help = "/Users/sophi529/Desktop/output/help_186.dat";
 
    
    LeggedAgent Insect;
    

    
    TSearch s(vector_size);

 
    //if(Model1)
        //s.SetVectorSize(vector_size_Model1);
    //else
        //TSearch s(vector_size);
    //cout << s.VectorSize() << endl;

    // Configure the search



    s.SetRandomSeed(seed);


    s.SetEvaluationFunction(evaluate);
    s.SetBestActionFunction(DumpCircuit);
    s.SetSelectionMode(RANK_BASED);
    s.SetReproductionMode(GENETIC_ALGORITHM);
    s.SetPopulationSize(150);
    s.SetMaxGenerations(8000);
    s.SetMutationVariance(0.1);
    s.SetCrossoverProbability(0.0);
    s.SetCrossoverMode(UNIFORM);
    s.SetMaxExpectedOffspring(1.1);
    s.SetElitistFraction(0.01);
    s.SetSearchConstraint(1);
    s.SetCheckpointInterval(5);

    // Run the search
    s.ExecuteSearch();
    

    //
 
    // Load the CTRNN into the agent
    char fname[] = "/Users/sophi529/Desktop/best.ns";
    ifstream ifs;
    ifs.open(fname);
    if (!ifs) {
        cerr << "File not found: " << fname << endl;
        exit(EXIT_FAILURE);
    }
    ifs >> Insect.NervousSystem;
    
    
    
    //randomize the neuron states. very slightly

    // Run the agent
    Insect.NervousSystem.RandomizeCircuitState(0,0.1);
    Insect.Reset(0, 0, 1);
    
    
    ofstream walkstream;
    walkstream.open(walk);
    ofstream helpstream;
    helpstream.open(help);
    for (double time = 0; time < RunDuration; time += StepSize) {
        Insect.Step(StepSize);
        helpstream << time << endl;
        for (int i = 0; i <=5; i++) {
            helpstream << "Joint X: " << Insect.LegVec[i].JointX << "Joint Y: " << Insect.LegVec[i].JointY << "Foot X: ";
            helpstream << Insect.LegVec[i].FootX << "Foot Y: " << Insect.LegVec[i].FootY << "FootState: ";
            helpstream << Insect.LegVec[i].FootState << endl;
            helpstream << "LegVec[i].Angle: " << Insect.LegVec[i].Angle << endl;
            
            
            walkstream << Insect.LegVec[i].JointY <<  " " << Insect.LegVec[i].JointX << " " ;
            walkstream << Insect.LegVec[i].FootY << " " << Insect.LegVec[i].FootX << " ";
            walkstream << Insect.LegVec[i].FootState << " ";
    }
        walkstream << endl;
        helpstream << endl;
        
       
    }
    ofstream infostream;
    infostream.open(info);
    infostream << "Seed: " << seed << endl;
    infostream << "Average velocity = " << Insect.LegVec[2].JointY/RunDuration << endl;
    infostream << "MaxLegForce6 = 0.05" << endl;
    infostream << "MaxLegForce1 = 0.75" << endl;
    infostream << "7 neurons" << endl;
    infostream << "all checking functions in place" << endl;
    infostream << "model 4" << endl;
    infostream << "population = 150" << endl;
    infostream << "generations = 8000" << endl;



  



     walkstream.close();
    infostream.close();
    helpstream.close();
    
    
    //// RETURN THIS
    // Display the fitness
    cout << "Average velocity = " << Insect.LegVec[2].JointY/RunDuration << endl;
    cout << "Random seed: " << seed << endl;
    //xl << "Average velocity = " << Insect.LegVec[0].JointY/RunDuration << endl;
    //xl.close();
    
    // Finished
    return 0;

}


/*
int main()
{
    TVector<double> testvec;
    testvec.SetSize(vector_size);
    for (int i = 1; i <=vector_size; i++) {
        testvec[i] = i;
    }
    
    LeggedAgent Insect;
    Insect.NervousSystem.SetCircuitSize(neuron_num);
    Assign_params(testvec, Insect.NervousSystem);
    cout << Insect.NervousSystem;
    return 0;
}
*/


/*
int main()
{
    //TSearch s(vector_size);
    LeggedAgent Insect;
    char fname[] = "/Users/sophi529/Desktop/cpg.ns";
    ifstream ifs;
    ifs.open(fname);
    if (!ifs) {
        cerr << "File not found: " << fname << endl;
        exit(EXIT_FAILURE);
    }
    ifs >> Insect.NervousSystem;
     Insect.NervousSystem.RandomizeCircuitState(0,0);
    ofstream output("/Users/sophi529/Desktop/output.dat");
     Insect.Reset(0, 0, 1);
        for (double time = 0; time < RunDuration; time += StepSize) {
            Insect.Step(StepSize);
            for (int i= 0 ; i <=5; i++) {
             
            output << Insect.LegVec[i].JointY << " " << Insect.LegVec[i].JointX << " " << Insect.LegVec[i].FootY << " " << Insect.LegVec[i].FootX << " "  << Insect.LegVec[i].FootState << " ";
            }
            output << endl;
        }
  //  Insect.Reset(0, 0);
    //Insect.GetFoot();
    //Insect.GetLegs();
    ifs.close();
    return 0;

}
*/
/*
 void DumpCircuit(int, TVector<double> &v)
 {
 CTRNN c(neuron_num);
 Assign_params(v,c);
 
 ofstream f("/Users/sophi529/Desktop/best.ns");
 f << c;
 }
 
 double evaluate(TVector<double> &v, RandomState &r){
 
 LeggedAgent Insect;
 
 Insect.NervousSystem.SetCircuitSize(neuron_num);
 Assign_params(v, Insect.NervousSystem);
 Insect.NervousSystem.RandomizeCircuitState(0,0,r);
 
 
 Insect.Reset(0, 0, 0, r);
 for (double time = 0; time < RunDuration; time += StepSize) {
 Insect.Step(StepSize);
 }
 
 return Insect.cx/RunDuration;
 
 }
 
 
 // The main program
 int main(int argc, char* argv[])
 {
 //
 LeggedAgent Insect;
 
 //
 
 
 //
 TSearch s(vector_size);
 
 // Configure the search
 s.SetRandomSeed(36574);
 s.SetEvaluationFunction(evaluate);
 s.SetBestActionFunction(DumpCircuit);
 s.SetSelectionMode(RANK_BASED);
 s.SetReproductionMode(GENETIC_ALGORITHM);
 s.SetPopulationSize(500);
 s.SetMaxGenerations(1000);
 s.SetMutationVariance(0.1);
 s.SetCrossoverProbability(0.0);
 s.SetCrossoverMode(UNIFORM);
 s.SetMaxExpectedOffspring(1.1);
 s.SetElitistFraction(0.01);
 s.SetSearchConstraint(1);
 s.SetCheckpointInterval(5);
 
 // Run the search
 s.ExecuteSearch();
 //
 // Load the CTRNN into the agent
 char fname[] = "/Users/sophi529/Desktop/best.ns";
 ifstream ifs;
 ifs.open(fname);
 if (!ifs) {
 cerr << "File not found: " << fname << endl;
 exit(EXIT_FAILURE);
 }
 ifs >> Insect.NervousSystem;
 
 
 
 
 
 // Run the agent
 Insect.NervousSystem.RandomizeCircuitState(0,0);
 Insect.Reset(0, 0, 0);
 ofstream xl;
 xl.open ("/Users/sophi529/Documents/Non college/2016 Summer/Indiana/Lab stuff/Generation_num/best_1000_36574.csv", ios::app);
 for (double time = 0; time < RunDuration; time += StepSize) {
 Insect.Step(StepSize);
 cout << Insect.Leg.JointX << " " << Insect.Leg.JointY << " ";
 cout << Insect.Leg.FootX << " " << Insect.Leg.FootY << " ";
 cout << Insect.Leg.FootState << endl;
 
 
 xl << Insect.Leg.JointX << ", " << Insect.Leg.JointY << ", ";
 xl << Insect.Leg.FootX << ", " << Insect.Leg.FootY << ", ";
 xl << Insect.Leg.FootState << endl;;
 }
 
 // Display the fitness
 cout << "Average velocity = " << Insect.cx/RunDuration << endl;
 xl << "Average velocity = " << Insect.cx/RunDuration << endl;
 xl.close();
 // Finished
 return 0;
 }
 */

