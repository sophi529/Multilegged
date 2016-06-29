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
//I FIXED THE NUMBER IN THE SEARCH VECTOR AND INCREASED THE INDIVIDUAL STANCE LEG MAX FORCE

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
const bool Model1 = true;
//all six connections evolved
const bool Model2 = false;
//contralateral connections evolved
const bool Model3 = false;
//ipsilateral connections evolved
const bool Model4 = false;



//CHANGED

int neuron_num = 30;
int onelegneuron_num = 5;

double vector_fill = 2;


/*double*/int vector_size = (neuron_num * neuron_num) + (2 * neuron_num);
int vector_size_Model1 = (onelegneuron_num * onelegneuron_num) + (2 * onelegneuron_num) + 12;
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
      
  {for(int i = 1;i <= onelegneuron_num;i++)
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
     
  else
  {for(int i = 1;i <= neuron_num;i++)
 NervousSystem.SetNeuronBias(i, MapSearchParameter(v[vi++], min_bias, max_bias));
 
 for(int i = 1;i <= neuron_num;i++) {
 double tau = MapSearchParameter(v[vi++], min_time, max_time);
 NervousSystem.SetNeuronTimeConstant(i, tau);
 }
//for(int i = 1; i <= neuron_num; i++){
 //   NervousSystem.SetNeuronGain(i, MapSearchParameter(v[vi++], min_bias, max_bias));
//}
  }
     
     
     
     
     if(!Model1)
     {
     //interconnections in one leg
     for(int i = 0;i <= (onelegneuron_num - 1);i++){
         for(int j = 0; j <= (onelegneuron_num - 1); j++){
             NervousSystem.SetConnectionWeight(leg1[i], leg1[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg2[i], leg2[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg3[i], leg3[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg4[i], leg4[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg5[i], leg5[j], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg6[i], leg6[j], MapSearchParameter(v[vi++], min_weight, max_weight));
         }}
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
         }}
         
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
        

     }
     
     //the legs are completely interconnected (contra and ipsalaterally)
     if(Model2) {
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg1[i], leg2[i], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg1[i], leg6[i], MapSearchParameter(v[vi++], min_weight, max_weight));
     }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg2[i], leg1[i], NervousSystem.ConnectionWeight(leg1[i], leg2[i]));
             NervousSystem.SetConnectionWeight(leg2[i], leg3[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg3[i], leg2[i], NervousSystem.ConnectionWeight(leg2[i], leg3[i]));
             NervousSystem.SetConnectionWeight(leg3[i], leg4[i], MapSearchParameter(v[vi++], min_weight, max_weight));
             NervousSystem.SetConnectionWeight(leg3[i], leg6[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg4[i], leg3[i], NervousSystem.ConnectionWeight(leg3[i], leg4[i]));
             NervousSystem.SetConnectionWeight(leg4[i], leg5[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
             NervousSystem.SetConnectionWeight(leg5[i], leg4[i], NervousSystem.ConnectionWeight(leg4[i], leg5[i]));
             NervousSystem.SetConnectionWeight(leg5[i], leg6[i], MapSearchParameter(v[vi++], min_weight, max_weight));
         }
         for(int i = 0;i <= onelegneuron_num - 1;i++){
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
    
    LeggedAgent Insect;

    
    TSearch s(2);
 
 
    //if(Model1)
        s.SetVectorSize(vector_size_Model1);
    //else
 
        //s.SetVectorSize(vector_size);
    //cout << s.VectorSize() << endl;

    // Configure the search



    s.SetRandomSeed(seed);


    s.SetEvaluationFunction(evaluate);
    s.SetBestActionFunction(DumpCircuit);
    s.SetSelectionMode(RANK_BASED);
    s.SetReproductionMode(GENETIC_ALGORITHM);
    s.SetPopulationSize(100);
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
    
    
    
    //randomize the neuron states. very slightly

    // Run the agent
    Insect.NervousSystem.RandomizeCircuitState(0,0.1);
    Insect.Reset(0, 0, 1);
    
    
    ofstream walkstream;
    walkstream.open(walk);

    for (double time = 0; time < RunDuration; time += StepSize) {
        Insect.Step(StepSize);
        
        for (int i = 0; i <=5; i++) {
            //cout << Insect.LegVec[i].JointX << " " << Insect.LegVec[i].JointY << " ";
            //cout << Insect.LegVec[i].FootX << " " << Insect.LegVec[i].FootY << " ";
            //cout << Insect.LegVec[i].FootState << endl;
            //cout << "LegVec[i].Angle: " << Insect.LegVec[i].Angle << endl;
            walkstream << Insect.LegVec[i].JointY <<  " " << Insect.LegVec[i].JointX << " " ;
            walkstream << Insect.LegVec[i].FootY << " " << Insect.LegVec[i].FootX << " ";
            walkstream << Insect.LegVec[i].FootState << " ";
    }
        walkstream << endl;
        
       
    }
    ofstream infostream;
    infostream.open(info);
    infostream << "Seed: " << seed << endl;
    infostream << "Average velocity = " << Insect.LegVec[2].JointY/RunDuration << endl;
    infostream << "I INCREASED THE INDIVIDUAL STANCE LEG MAX FORCE" << endl;

     walkstream.close();
    infostream.close();
    
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

