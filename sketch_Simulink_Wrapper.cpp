#include <iostream>
#include <chrono>

#include <thread>
#include <math.h>
#include <complex>



double i_Lambda = 0;
auto t_prev_Lambda = std::chrono::high_resolution_clock::now();
int SoC_error_sign = 0;

// TODO: get system time.
double Updating_Lambda (const double Lambda_const, 
                        const double Lambda_gain, 
                        const double SoC_desired, 
                        const double SoC,
                        auto& t_prev,
                        const double k_i,
                        double& i_Lambda,
                        int& SoC_error_sign) {
    // t_prev: time stamp from last update of Lambda
    // k_i: integrator gain
    // SoC_error_sign: Anti-windup reset memory


    auto t = std::chrono::high_resolution_clock::now(); // get current system time
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t - t_prev); // get time difference
    t_prev = t; // update the time
    double dt = duration.count()/(1000*1000); // calculate the dt with resolution of microseconds converted to second
    i_Lambda += dt * (SoC_desired - SoC); // add it to integral
    
    // Anti-windup integral releast condition
    if ((SoC_desired - SoC) > 0) {
        // Error sign positiv, check if memory sign mismatch
        if (SoC_error_sign <= 0) {
            // Sign changed, reset integral
            i_Lambda = 0;
            SoC_error_sign = 1;
        }
    } else if ((SoC_desired - SoC) < 0) {
        // Error sign negativ, check if memory sign mismatch
        if (SoC_error_sign >= 0) {
            // Sign changed, reset integral
            i_Lambda = 0;
            SoC_error_sign = -1;
        }
    }


    double Lambda = Lambda_const + Lambda_gain * (SoC_desired - SoC) + k_i * i_Lambda;
    //std::cout << i_Lambda << "\n";
    return Lambda;
}


double find_P_Motor_req (const double P_Pilot_req, const double P_FC_actual, const double P_BAT_UB, const double P_BAT_LB, const double P_AUX) {
    // P_AUX is including P_Comp and all the other power consumption.
    double P_Motor_max = P_FC_actual + P_BAT_UB - P_AUX; // Get Motor UB
    double P_Motor_min = P_FC_actual + P_BAT_LB - P_AUX; // Get Motor LB

    if (P_Motor_min <= 0) {
        // incase LB is smaller than 0
        P_Motor_min = 0; // set the LB to 0
    }

    // initialize Motor power request and set this to LB
    double P_Motor_req = P_Motor_min;

    // Now consider the Pilot request level
    if (P_Motor_min < P_Pilot_req) {
        // Pilot want more than LB
        P_Motor_req = P_Pilot_req;

        if (P_Motor_max < P_Pilot_req) {
            // Pilot want more than UP, system can't supply -> pilot request rejected and set to UB
            P_Motor_req = P_Motor_max;
        }
    }
    // otherwiese Pilot request is lower than LB, request rejected and Motor request remains LB
    
    return P_Motor_req;
}



// Input preprocessing -------------------------------------
struct Input_processing {
    double P_req_corr, P_BAT_min_req, P_BAT_UB, P_BAT_LB;
};

Input_processing P_req_limiter (const double P_Pilot_req, 
                                const double P_AUX, 
                                const double SoC, 
                                const double P_FC_max,
                                const double SoC_UB, 
                                const double SoC_LB, 
                                const double BMS_P_UB, 
                                const double BMS_P_LB) {
    // This block correct the power request add up the AUX power while consider the Power limits as well
    // P_Pilot_req is raw from pilot
    // P_AUX is including compressor and other
    // P_FC_max is the maximum FC designed power, FC after DC-DC raw power, compressor power is not substracted
    // BMS_P_UB,LB is the BMS value
    Input_processing P;

    // SoC & P_UB & P_LB are the value feed from the BMS.
    // SoC_UB & SoC_LB are user defined value.
    if (SoC_LB <= SoC) {
        // If SoC in bound, can draw, use BMS value.
        P.P_BAT_UB = BMS_P_UB;
    } else {
        P.P_BAT_UB = 0.0001; // "Bleed power" to avoid program crashing in Simulink
    }
    if (SoC <= SoC_UB) {
        // If SoC in bound, can charge, use BMS value.
        P.P_BAT_LB = BMS_P_LB;
    } else {
        P.P_BAT_LB = -0.0001; // "Bleed power" to avoid program crashing in Simulink
    }

    double P_req_tot = P_Pilot_req + P_AUX; // calculate the total power request
    double P_max_combined = P_FC_max + P.P_BAT_UB; // Theoretical maximum reachable power output of whole system

    // The following condition make the power request within bound
    if (P_max_combined <= P_req_tot) {
        P.P_req_corr = P_max_combined;
    } else if (P_req_tot <= 0) {
        P.P_req_corr = 0;
    } else {
        P.P_req_corr = P_req_tot;
    }

    // The following determin min Battery power, so when the power request is higher than FC, battery must suppy a minimum amount of power.
    P.P_BAT_min_req = P.P_req_corr - P_FC_max;
    if (P.P_BAT_min_req <= 0) {
        P.P_BAT_min_req = 0;
    }
    return P;
}

// ---------------------------------------------------------


struct Power_Split {
    double P_FC_req, P_BAT_req;
};

Power_Split run_split (const double P_req_corr, 
                       const double Q, 
                       const double P_BAT_UB, 
                       const double P_BAT_LB, 
                       const double P_FC_max) {

    // P_BAT_UB,LB must passed the SoC limiter in Inputprocessing
    
    Power_Split P;

    // Definition of Q:
    // To get P_BAT_req
    if (0 <= Q) {
        P.P_BAT_req = Q * P_BAT_UB;
    } else {
        P.P_BAT_req = - Q * P_BAT_LB;
    }

    // This code is different than in Simulink, we assume that the SOC_limiter already takes care of the P_BAT_UB,LB
    // So the limiter block here is not included

    // P_FC_req
    P.P_FC_req = P_req_corr - P.P_BAT_req;

    // limitation:
    if (P.P_FC_req <= 0) {
        P.P_FC_req = 0;
    } else if (P_FC_max <= P.P_FC_req) {
        //P.P_FC_req = P_FC_max; removed UB for FC
    }

    return P;
}



// Internal FC Model
struct P_FC_2_dm_and_P_Comp_model {
    double dm_H2, P_Comp;
};

P_FC_2_dm_and_P_Comp_model run_FC_model (const double P_FC) {

    P_FC_2_dm_and_P_Comp_model Value; // initiating a Struct with different values saved in

    // FC quadratic model
    double a1 = 7.95454545454545e-14,
    b1 = 1.48851010101010e-08,
    c1 = 6.67424242424242e-05;
    
    Value.dm_H2 = a1 * (P_FC*P_FC) + b1 * P_FC + c1;

    // Comp power demand model
    double a2 = 6.55497412831353e-07,
    b2 = 0.110383876465320,
    c2 = 7821.39907716565;

    Value.P_Comp = a2 * (P_FC*P_FC) + b2 * P_FC + c2;

    return Value;
}


// Internal BAT Model
struct BAT_Model {
    double R_i, u_oc, dSoC, I_BAT, P_BAT_max;
    bool overload;
    // R_i      : Internal Resistance, depends on SoC
    // u_oc     : Open Circle Voltage
    // dSoC     : Rate of change from SoC, (SoC,Q_0)
    // I_BAT    : Battery current under a load 
    // overload : True -> Battery can not supply the requested power
};

BAT_Model run_BAT_model (const double SoC,
                         const double Q_0,
                         const double P_BAT) {
    // SoC: current State of Charge
    // Q_0: Total capacity of BAT in [C]

    BAT_Model BAT; // initializing Struct to save values


    // SoC_2_R_i (SoC)
    // PWA aproximation of R_i
    // R_i in [Ohm]
    // Data point
    const double SoC_0 = 0;
    const double R_i_0 = 9;

    const double SoC_1 = 0.2;
    const double R_i_1 = 3.6;

    const double SoC_2 = 0.4;
    const double R_i_2 = 2.7;

    const double SoC_3 = 0.8;
    const double R_i_3 = 2.7;

    const double SoC_4 = 1;
    const double R_i_4 = 1.8;

    if (SoC_0 <= SoC && SoC < SoC_1) {
        BAT.R_i = R_i_0 + ((R_i_1 - R_i_0)/(SoC_1-SoC_0))*(SoC-SoC_0);
    }else if (SoC_1 <= SoC && SoC < SoC_2) {
        BAT.R_i = R_i_1 + ((R_i_2 - R_i_1)/(SoC_2-SoC_1))*(SoC-SoC_1);
    }else if (SoC_2 <= SoC && SoC < SoC_3) {
        BAT.R_i = R_i_2 + ((R_i_3 - R_i_2)/(SoC_3-SoC_2))*(SoC-SoC_2);
    }else if (SoC_3 <= SoC && SoC <= SoC_4) {
        BAT.R_i = R_i_3 + ((R_i_4 - R_i_3)/(SoC_4-SoC_3))*(SoC-SoC_3);
    }else {BAT.R_i = -1;}

    // Internal Resistance calculation completed and stored @ BAT.R_i


    //SoC_2_u_oc(SoC)
    // PWA approximation
    // u_oc in [V]
    // Data point
    const double SoC_0a = 0;
    const double u_oc_0 = 540;

    const double SoC_1a = 0.2;
    const double u_oc_1 = 630;

    const double SoC_2a = 0.8;
    const double u_oc_2 = 666;

    const double SoC_3a = 0.93;
    const double u_oc_3 = 720;

    const double SoC_4a = 1;
    const double u_oc_4 = 756;
 
    if (SoC_0a <= SoC && SoC < SoC_1a) {
        BAT.u_oc = u_oc_0 + ((u_oc_1 - u_oc_0)/(SoC_1a-SoC_0a))*(SoC-SoC_0a);
    }else if (SoC_1a <= SoC && SoC < SoC_2a) {
        BAT.u_oc = u_oc_1 + ((u_oc_2 - u_oc_1)/(SoC_2a-SoC_1a))*(SoC-SoC_1a);
    }else if (SoC_2a <= SoC && SoC < SoC_3a) {
        BAT.u_oc = u_oc_2 + ((u_oc_3 - u_oc_2)/(SoC_3a-SoC_2a))*(SoC-SoC_2a);
    }else if (SoC_3a <= SoC && SoC <= SoC_4) {
        BAT.u_oc = u_oc_3 + ((u_oc_4 - u_oc_3)/(SoC_4a-SoC_3a))*(SoC-SoC_3a);
    }else {BAT.u_oc = -1;}

    // Open Circle Votalge @ BAT.u_oc

    // Following function already consider the maximum power, even if the P_BAT is higher than max, current will be limited
    BAT.I_BAT = std::real((BAT.u_oc - sqrt(std::complex<double>(BAT.u_oc*BAT.u_oc - 4*BAT.R_i*P_BAT)))/(2*BAT.R_i));
    BAT.dSoC = - BAT.I_BAT/Q_0;

    BAT.P_BAT_max = (BAT.u_oc*BAT.u_oc)/(4*BAT.R_i);

    if (BAT.P_BAT_max < P_BAT) {
        BAT.overload = true;
    } else {BAT.overload = false;}

    return BAT;
}


//

double find_optimum_Q (const double P_req_corr,
                       const double P_FC_max,
                       const double Lambda,
                       const double P_BAT_UB,
                       const double P_BAT_LB,
                       const double SoC,
                       const double Q_0) {
    
    // P_BAT_LB,UB must passed the SoC limiter (input processing)

    int rows = 2;
    int cols = 201; //default 201
    double Q[rows][cols]; // 2 rows, 201 step resolution
    double gap = 2.0/((double)cols - 1);

    // Make a 2D matrix first row saving the Q from -1 to 1
    // Second row save the value of Hamiltonian values
    for (int i = 0; i < cols; ++i) {
        // Span the Q from -1 to 1
        Q[0][i] = i*(gap) - 1 ; // First row to save Q span
        // Feed this current Q into power split
        Power_Split Split = run_split (P_req_corr, Q[0][i], P_BAT_UB, P_BAT_LB, P_FC_max);
        // we obtain P_FC_req and BAT_req with the Q

        // run FC model to get the hydrogen consumption:
        P_FC_2_dm_and_P_Comp_model FC = run_FC_model (Split.P_FC_req);

        // run BAT model get d_SoC:
        BAT_Model BAT = run_BAT_model (SoC,Q_0,Split.P_BAT_req);
        
        // save the value of Hamiltonian into the second row:
        Q[1][i] = FC.dm_H2 - Lambda * BAT.dSoC; 
    }

    // Find the minimum in the second row
    double minValue = std::numeric_limits<double>::max(); // initialization of the maximum possible
    double Q_final = -10;

    for (int i = 0; i < cols; ++i) {
        if (Q[1][i] < minValue) {
            // Whenever it encounters a new minimum, update the value & the Q
            minValue = Q[1][i];
            Q_final = Q[0][i];
        }
    }

    return Q_final;
}


double finalizing_power_split (const double Q,
                               const double P_BAT_UB,
                               const double P_BAT_LB,
                               const double P_BAT_min_req,
                               const double P_FC_max,
                               const double P_req_corr) {
    //
    

    // Power Split:
    Power_Split P = run_split (P_req_corr,Q,P_BAT_UB,P_BAT_LB,P_FC_max);
    
    if (0 < P_BAT_min_req) {
        // Check the Q suggestion if it satisfied.
        if (P.P_BAT_req < P_BAT_min_req) {
            // If the suggested power request is smalelr than the minimum, set it to minimum.
            P.P_BAT_req = P_BAT_min_req;
        } // If suggested request higher than the minimum, take the suggest.
    }
    // else use the Q for power split

    // pass through limiter again
    if (P.P_BAT_req < P_BAT_LB) {
        P.P_BAT_req = P_BAT_LB;
    } else if (P_BAT_UB < P.P_BAT_req){
        P.P_BAT_req = P_BAT_UB;
    }

    // Power balance
    double P_balance_1 = P_req_corr - P.P_BAT_req; // What FC should supply 1
    // Edge case whe Q = -1 and want to charg BAT, but FC can not supply full charging power
    if (P_FC_max < P_balance_1) {
        // Higher than FC can supply, run FC at max, supply the power request and the rest charg BAT
        P.P_BAT_req = P_req_corr - P_FC_max;
    }
    // else nothing change

    double P_FC_req = P_req_corr - P.P_BAT_req;

    // Final Limiting P_FC_req
    if (P_FC_req <= 0) {
        return P_FC_req = 0;
    } else if (P_FC_max <= P_FC_req) {
        return P_FC_req = P_FC_max;
    } else {
        return P_FC_req;
    }
}




// Debugging

/*
int main () {
    double P_Pilot_req, P_AUX, SoC, P_FC_max, SoC_UB, SoC_LB, P_UB, P_LB, P_FC, P_BAT, Q_0;
    P_Pilot_req = 0;
    P_AUX = 500;
    SoC_UB = 0.9;
    SoC = 0.3;
    SoC_LB = 0.2;
    P_FC_max = 95000;
    P_UB = 30000;
    P_LB = -10000;
    P_FC = 50000;
    P_BAT = 27562.5;
    Q_0 = 32400;

    BAT_Model Battery = run_BAT_model (SoC,Q_0,P_BAT);
    std::cout << "dSoC: " << Battery.dSoC << "\n";
    std::cout << "I_BAT: " << Battery.I_BAT << "\n";
    std::cout << "overload: " << Battery.overload << "\n";
    std::cout << "P_BAT_max: " << Battery.P_BAT_max << "\n";
    std::cout << "R_i: " << Battery.R_i << "\n";
    std::cout << "u_oc: " << Battery.u_oc << "\n";


    
    Input_processing Limater = P_req_limiter (P_Pilot_req, P_AUX, SoC, P_FC_max, SoC_UB, SoC_LB, P_UB, P_LB);
    std::cout << "P_BAT_min_req: " << Limater.P_BAT_min_req << "\n";
    std::cout << "P_req_limited: " << Limater.P_req_corr << "\n";
    std::cout << "Battery UB: " << Limater.P_BAT_UB << "\n";
    std::cout << "Battery UB: " << Limater.P_BAT_UB << "\n";
    std::cout << "Battery LB: " << Limater.P_BAT_LB << "\n";

    P_FC_2_dm_and_P_Comp_model Value = run_FC_model (P_FC);
    std::cout << "dm_H2: " << Value.dm_H2 << "\n";
    std::cout << "P_Comp: " << Value.P_Comp << "\n";

    double Totoal = P_FC - Value.P_Comp;
    std::cout << "Total: " << Totoal << "\n";


    double Q = find_optimum_Q (20000,P_FC_max,0.43,Limater.P_BAT_UB,Limater.P_BAT_LB,SoC,Q_0);



    std::cout << "Min: " << Q << "\n";




    return 0;
}
*/

void Simulink_Wrapper(const double P_Pilot_req,
                      const double P_AUX,
                      const double SoC,
                      const double P_FC,
                      const double BMS_UB,
                      const double BMS_LB,
                      int& SoC_error_sign,
                      auto& t_prev_Lambda,
                      double& i_Lambda) {

    double SoC_UB, SoC_LB, P_FC_max;
    SoC_UB = 0.9;
    SoC_LB = 0.2;

    // Feed in all value and correct the power request -> Dead zone of throttle
    Input_processing Limiter = P_req_limiter (P_Pilot_req,
                                              P_AUX,SoC,
                                              P_FC_max,
                                              SoC_UB,
                                              SoC_LB,
                                              BMS_UB,
                                              BMS_LB);

    double Lambda_const = 0.3;
    double Lambda_gain = 0.1;
    double SoC_desired = 0.65;
    double k_i = 0.001;


    double Lambda = Updating_Lambda (Lambda_const,
                                     Lambda_gain, 
                                     SoC_desired, 
                                     SoC, 
                                     t_prev_Lambda, 
                                     k_i, 
                                     i_Lambda, 
                                     SoC_error_sign);

    double Q = find_optimum_Q (Limiter.P_req_corr,
                               P_FC_max,
                               Lambda,
                               Limiter.P_BAT_UB,
                               Limiter.P_BAT_LB,
                               SoC,
                               Q_0);

    double P_FC_req = finalizing_power_split (Q,
                                              Limiter.P_BAT_UB,
                                              Limiter.P_BAT_LB,
                                              Limiter.P_BAT_min_req,
                                              P_FC_max,
                                              Limiter.P_req_corr);
    
    double P_Motor = find_P_Motor_req (P_Pilot_req,
                                       P_FC,
                                       Limiter.P_BAT_UB,
                                       Limiter.P_BAT_LB,
                                       P_AUX);
}


int main () {

    double i_Lambda = 0.325;
    auto t_prev_Lambda = std::chrono::high_resolution_clock::now();
    int SoC_error_sign = 1;

    double P_Pilot_req, P_AUX, SoC, P_FC_max, SoC_UB, SoC_LB, BMS_UB, BMS_LB, P_FC,  Q_0;
    P_Pilot_req = 0;    // Throttle level
    P_AUX = 0;          // All power demand incl. P_comp
    P_FC_max = 95000;   // FC design maximum
    SoC_UB = 0.9;       // User defined value
    SoC = 0.7;          // External signal
    SoC_LB = 0.2;       // User defined value
    BMS_UB = 30000;       // BMS value
    BMS_LB = -10000;      // BMS value
    P_FC = 90000;       // P_FC_actual, mesurement from the sensor
              // P_BAT_actual, mesurement from the sensor
    Q_0 = 32400;

    // Feed in all value and correct the power request -> Dead zone of throttle
    Input_processing Limiter = P_req_limiter (P_Pilot_req,
                                              P_AUX,SoC,
                                              P_FC_max,
                                              SoC_UB,
                                              SoC_LB,
                                              BMS_UB,
                                              BMS_LB);
    
    double Lambda_const = 0.3;
    double Lambda_gain = 0.1;
    double SoC_desired = 0.65;
    double k_i = 0.001;

    double Lambda = Updating_Lambda (Lambda_const,
                                     Lambda_gain, 
                                     SoC_desired, 
                                     SoC, 
                                     t_prev_Lambda, 
                                     k_i, 
                                     i_Lambda, 
                                     SoC_error_sign);

    std::cout << "Lambda: " << Lambda << "\n";

    double Q = find_optimum_Q (Limiter.P_req_corr,
                               P_FC_max,
                               Lambda,
                               Limiter.P_BAT_UB,
                               Limiter.P_BAT_LB,
                               SoC,
                               Q_0);

    std::cout << "Q: " << Q << "\n";

    double P_FC_req = finalizing_power_split (Q,
                                              Limiter.P_BAT_UB,
                                              Limiter.P_BAT_LB,
                                              Limiter.P_BAT_min_req,
                                              P_FC_max,
                                              Limiter.P_req_corr);


    std::cout << "P_FC_req: " << P_FC_req << "\n";


    double P_Motor = find_P_Motor_req (P_Pilot_req,
                                       P_FC,
                                       Limiter.P_BAT_UB,
                                       Limiter.P_BAT_LB,
                                       P_AUX);

    std::cout << "P_Motor: " << P_Motor << "\n";

    return 0;
}

