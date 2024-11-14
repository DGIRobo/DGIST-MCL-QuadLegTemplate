#include "filter.h"

double Ts = 0.0001;
double pi = 3.141592;

double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
  // input, input_old, output_old (이전 tustin 이산 미분 결과물)을 받아서 tustin 이산 미분을 계산하고 이번 step에서의 input의 tustin 이산 미분 결과값을 output으로 리턴하는 함수
    double time_const = 1 / (2 * pi * cutoff_freq);
    // 시상수 tau(time_constant) 계산
    double output = 0;
    // tustin_derivative의 결과값(output)을 저장할 공간 할당

    output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);
    // tustin_derivative 계산

    return output;
    // tustin_derivativ 결과값(output) 출력
}

double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq)
{
  // input, input_old, output_old (이전 lowpassfilter 결과물)을 받아서 현재 step에서의 lowpassfilter 결과물을 계산하고 리턴하는 함수
    //double cutoff_freq = 100;
    double time_const = 1 / (2 * pi * cutoff_freq);
    // 시상수 tau(time_constant) 계산

    double output = 0;
    // lowpassfilter의 결과값(output)을 저장할 공간 할당

    output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);
    // lowpassfilter 계산

    return output;
    // lowpassfilter 결과값(output) 출력
}
