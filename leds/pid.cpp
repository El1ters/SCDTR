#include "pid.h"
#include <Arduino.h>

pid::pid( float _h, float _K, float b_,
 float Ti_, float Td_, float N_,float _Tt)
 // member variable initialization list
 : h {_h}, K {_K}, b {b_}, Ti {Ti_}, Td {Td_},
 N {N_}, I {0.0}, D {0.0}, Tt{_Tt}
 { antiwindup = 1, feedback = 1,occupancy = 0, active = 1, Kold = K, bold = b;} // should check arguments validity

float pid::compute_control(float r, float y,float h) {
    float P;
    float u;
    if (get_feedback() == true){
        //float ad = Td/(Td+N*h);
        //float bd = Td*K*N/(Td+N*h);
        float bi = K*h/Ti;
        float ao = h/Tt;
        //Serial.printf("Ti: %f Tt: %f\n",Ti,Tt);
        //Serial.printf("bi: %f ao: %f\n",bi,ao);
        float e = r - y;
        P = K*(b* r - y);
        //D = ad*D-bd*(y-y_old);
        I += Kold*(bold*e)- K * (b * e);    // update integral for bumpless transfer
        Kold = K, bold = b;
        float v = P+I;
        
        if (antiwindup == 0){ //sem anti-windup
            u = min(max(v, 0.0), 1); //saturation
            I += K*h/Ti*e; //sem anti-windup
        }else{ //anti-windup
            u = min(max(v, 0.0), 1); //saturation
            I += bi * e + ao * (u - v);
        }
    }else{
        P = K*b*r;
        u = P;
    } 
    return u;
}

bool pid::get_feedback() {
    return feedback;
}

void pid::set_feedback( int _feedback ) {
    feedback = _feedback;
}

bool pid::get_antiwindup() {
    return antiwindup;
}

void pid::set_antiwindup( int _antiwindup ) {
    antiwindup = _antiwindup;
}

bool pid::get_occupancy() {
    return occupancy;
}

void pid::set_occupancy( int _occupancy ) {
    occupancy = _occupancy;
}

bool pid::get_active() {
    return active;
}

void pid::set_active( int _active ) {
    active = _active;
}
