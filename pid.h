#ifndef PID_H
#define PID_H

class pid {
    float I, D, K, Ti, Td, b, h, y_old, N, Tt;
    double Kold, bold;
public:
    explicit pid( float _h, float _K = 1, float b_ = 1,
    float Ti_ = 1, float Td_ = 0, float N_ = 10, float _Tt = 100);
    ~pid() {};
    float compute_control( float r, float y, float h);
    bool get_feedback();
    void set_feedback( int _feedback );
    bool get_antiwindup();
    void set_antiwindup( int _antiwindup );

private:
    bool feedback, antiwindup ;
};
#endif //PID_H 
