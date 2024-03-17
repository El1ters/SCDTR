#ifndef PID_H
#define PID_H

class pid {
    
public:
    explicit pid( float _h, float _K = 1, float b_ = 1,
    float Ti_ = 1, float Td_ = 0, float N_ = 10, float _Tt = 100);
    ~pid() {};
    float compute_control( float r, float y, float h);
    bool get_feedback();
    void set_feedback( int _feedback );
    bool get_antiwindup();
    void set_antiwindup( int _antiwindup );
    bool get_occupancy();
    void set_occupancy( int _occupancy );
    bool get_active();
    void set_active( int _active );
    float I, D, K, Ti, Td, b, h, N, Tt, gain;
    double Kold, bold;

private:
    bool feedback, antiwindup, occupancy, active;
};
#endif //PID_H 
