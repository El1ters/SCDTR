#include "pid.h"
#include <Arduino.h>

pid::pid( float _h, float _K, float b_,
 float Ti_, float Td_, float N_,float _Tt)
 // member variable initialization list
 : h {_h}, K {_K}, b {b_}, Ti {Ti_}, Td {Td_},
 N {N_}, I {0.0}, D {0.0}, y_old{0.0}, Tt{_Tt}
 { antiwindup = 0, feedback = 0 ;} // should check arguments validity

float pid::compute_control(float r, float y,float h) {
    float P = K*(b*r-y);
    float ad = Td/(Td+N*h);
    float bd = Td*K*N/(Td+N*h);
    float bi = K*h/Ti;
    float ao = h/Tt;
    float u;
    float e = r - y;

    D = ad*D-bd*(y-y_old);
    float v = P+I+D;
    
    

    I += Kold*(bold*r-y)- K * (b * r-y);    // update integral for bumpless transfer
    Kold = K, bold = b;
    
    if (antiwindup == 0){ //sem anti-windup
        u = v;
        I += K*h/Ti*e; //sem anti-windup
    }else{ //anti-windup
        u = min(max(v, 0.0), 4096); //saturation
        I = I + bi*(e) + ao*(u - v);
    }
    

    if( u < 0 ) u = 0;
    if( u > 4095 ) u = 4095;
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

