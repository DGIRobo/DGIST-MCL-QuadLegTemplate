#include "controller.h"
#include <iostream>
using namespace std;


controller::controller()
{
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Pos PID 관련 값들 초기화
        Kp_pos[i] = 0.0;
        Kd_pos[i] = 0.0;

        error_pos[i] = 0.0;
        error_old_pos[i] = error_pos[i];
        error_dot_pos[i] = 0.0;
        error_dot_old_pos[i] = error_dot_pos[i];
        PID_output_pos[i] = 0;

        // Vel PID 관련 값들 초기화
        Kp_vel[i] = 0.0;
        Kd_vel[i] = 0.0;

        error_vel[i] = 0.0;
        error_old_vel[i] = error_vel[i];
        error_dot_vel[i] = 0.0;
        error_dot_old_vel[i] = error_dot_vel[i];
        PID_output_vel[i] = 0;

        //Admittance Control 관련 값들 초기화
        deltaPos[i] = 0.0;
        deltaPos_old[i] = deltaPos[i];
        deltaPos_old2[i] = deltaPos_old[i];

        //RWDOB 관련 값들 초기화
        rhs_dob[i] = 0.0;
        rhs_dob_old[i] = rhs_dob[i];
        lhs_dob[i] = 0.0;
        lhs_dob_old[i] = lhs_dob[i];
        lhs_dob_LPF[i] = 0.0;
        lhs_dob_LPF_old[i] = lhs_dob_LPF[i];
        rhs_dob_LPF[i] = 0.0;
        rhs_dob_LPF_old[i] = rhs_dob_LPF[i];
        tauDist_hat[i] = 0.0;

        //RWFOB 관련 값들 초기화
        rhs_fob[i] = 0.0;
        rhs_fob_old[i] = rhs_fob[i];
        lhs_fob_LPF[i] = 0.0;
        lhs_fob_LPF_old[i] = lhs_fob_LPF[i];
        rhs_fob_LPF[i] = 0.0;
        rhs_fob_LPF_old[i] = rhs_fob_LPF[i];
        tauExt_hat[i] = 0.0;
        forceExt_hat[i] = 0.0;
        forceExt_hat_old[i] = forceExt_hat[i];
        forceExt_hat_old2[i] = forceExt_hat_old[i];




    }

};
controller::~controller(){}

void controller::pid_gain_pos(double kp, double kd, double cut_off)
{
    for (int i = 0; i < NDOF_LEG; i++) // system을 이루는 모든 motor에 대해
    {
        Kp_pos[i] = kp; // Position PID controller의 Kp gain을 받은 Kp값으로 설정
        Kd_pos[i] = kd; // Position PID controller의 Kd gain을 받은 Kd값으로 설정
        cut_off_D_pos = cut_off; // Position PID controller의 D controller의 cutoff frequency를 받은 cut_off값으로 설정

    }

};

void controller::pid_gain_vel(double kp, double kd, double cut_off)
{
    for (int i = 0; i < NDOF_LEG; i++)
    {
        Kp_vel[i] = kp;
        Kd_vel[i] = kd;
        cut_off_D_vel = cut_off;

    }

};

void controller::ctrl_update()
{
    for (int i = 0; i < NDOF_LEG; i++)
    {
        //PID pos
        error_old_pos[i] = error_pos[i];
        error_dot_old_pos[i] = error_dot_pos[i];
        //PID vel
        error_old_vel[i] = error_vel[i];
        error_dot_old_vel[i] = error_dot_vel[i];

        //admittance
        deltaPos_old2[i] = deltaPos_old[i];
        deltaPos_old[i] = deltaPos[i];

        //DOB
        rhs_dob_old[i] = rhs_dob[i];
        lhs_dob_old[i] = lhs_dob[i];
        rhs_dob_LPF_old[i] = rhs_dob_LPF[i];
        lhs_dob_LPF_old[i] = lhs_dob_LPF[i];

        //FOB
        rhs_fob_LPF_old[i] = rhs_fob_LPF[i];
        lhs_fob_LPF_old[i] = lhs_fob_LPF[i];
        forceExt_hat_old2[i] = forceExt_hat_old[i];
        forceExt_hat_old[i] = forceExt_hat[i];


    }

};

Vector2d controller::PID_pos(StateModel_* state_model)
{
    for (int i = 0; i < NDOF_LEG; i++) // Error를 state 모델에 넣을 필요 있는지 생각해봐야함. error는 여기에 있어도 됨. //error들 update 해줘야함
    {
        error_pos[i] = state_model->posRW_ref[i] - state_model->posRW[i];
        // state_model이라는 class가 가진 값들을 참조해서 RW coordinate 상의 Foot의 현재 Position Error를 계산
        error_old_pos[i] = state_model->posRW_ref_old[i] - state_model->posRW_old[i];
        // state_model이라는 class가 가진 값들을 참조해서 RW coordinate 상의 Foot의 과거 Position Error를 계산

        error_dot_pos[i] = tustin_derivative(error_pos[i], error_old_pos[i], error_dot_old_pos[i], cut_off_D_pos);
        // 현재 Position error와 과거 Position errror 그리고 과거에 tustin_derivative로 계산한 Position error velocity를 토대로 현재의 tustin_derivative 계산한 Positin error velocity를 계산

        // DOB 끄면 PID만 사용해야하는데 state model에 넣지 않아도 되는지 생각해봐야함.
        PID_output_pos[i] = Kp_pos[i] * error_pos[i] + Kd_pos[i] * error_dot_pos[i]; // 이걸 return을 사용하면?
        // P Control과 D Control ouput을 합해서 Position PID Control의 output을 계산
    }
    return PID_output_pos;
    // Position PID Control의 output(r 방향 힘, theta_r 방향 힘)을 리턴
};

void controller::PID_vel(StateModel_* state_model)
{
    for (int i = 0; i < NDOF_LEG; i++)
    {
        error_vel[i] = state_model->velRW_ref[i] - state_model->velRW[i]; //error is no vector
        error_old_vel[i] = state_model->velRW_ref_old[i] - state_model->velRW_old[i];

        error_dot_vel[i] = tustin_derivative(error_vel[i], error_old_vel[i], error_dot_old_vel[i], cut_off_D_vel);

        // DOB 끄면 PID만 사용해야하는데 state model에 넣지 않아도 되는지 생각해봐야함.
        PID_output_vel[i] = Kp_vel[i] * error_vel[i] + Kd_vel[i] * error_dot_vel[i]; // 이걸 return을 사용하면?
    }
}; // negative velocity PID feedback

void controller::admittanceCtrl(StateModel_* state_model, double omega_n , double zeta, double k, int flag)
{
    // admittance controller를 사용함으로써 Robot Leg가 RW coordinate 상의 r-direction에서 MBK system처럼 행동하도록 만듬
    // 현재 omega_n, zeta, k 로 tunning 하고 있는데, 변환식을 통해 아래에 적어주면 된다
    ad_M = k/(pow(omega_n,2));
    ad_B = 2*zeta*k/omega_n;
    ad_K = k;

    // forceExt_hat = ad_M * delta_ddot + ad_B * delta_dot + ad_K * delta를 Tustin Discrete로 변환한 후에 delta에 대한 식으로 정리함
    double c1 = 4 * ad_M + 2 * ad_B * Ts + ad_K * pow(Ts, 2);
    double c2 = -8 * ad_M + 2 * ad_K * pow(Ts, 2);
    double c3 = 4 * ad_M - 2 * ad_B * Ts + ad_K * pow(Ts, 2);

    deltaPos[0] =
        (pow(Ts, 2) * forceExt_hat[0] + 2 * pow(Ts, 2) * forceExt_hat_old[0] +
            pow(Ts, 2) * forceExt_hat_old2[0] - c2 * deltaPos_old[0] - c3 * deltaPos_old2[0]) / c1;

    //cout<< " deltaPos: " << deltaPos[0]<< endl;

        if (flag == true)
            state_model->posRW_ref[0] = state_model->posRW_ref[0] + deltaPos[0];
            // state_model의 Rotation Coordinate에서 정의되는 Position Reference에 Admittance Controller를 통해 얻은 delta값을 더함.
};

Vector2d controller::DOBRW(StateModel_* state_model, double cut_off ,int flag)
{
    cut_off_dob = cut_off;
    // Disturbance Q filter를 받은 cut_off값으로 설정
    lhs_dob = state_model->tau_bi;
    // state_model이라는 class의 Biarticular Torque(Position PID의 output)를 참조해서 DOB를 위한 lhs에 할당
    rhs_dob = state_model->Lamda_nominal_DOB * state_model->qddot_bi_tustin;
    // state_model이라는 class의 tustin 변환한 biarticular joint의 Acceleration에 J^T * Lambda_n^R * J를 곱해서 DOB를 위한 rhs에 할당

    if (flag == true)
    {
        for (int i = 0; i < NDOF_LEG; i++)
        {
            lhs_dob_LPF[i] = lowpassfilter(lhs_dob[i], lhs_dob_old[i], lhs_dob_LPF_old[i], cut_off_dob);
            // lhs를 Q filter에 통과시켜서 lhs_LPF에 할당
            rhs_dob_LPF[i] = lowpassfilter(rhs_dob[i], rhs_dob_old[i], rhs_dob_LPF_old[i], cut_off_dob);
            // rhs를 Q filter에 통과시켜서 rhs_LPF에 할당

            tauDist_hat[i] = lhs_dob_LPF[i] - rhs_dob_LPF[i];
            // lhs_LPF에 rhs_LPF를 빼서 Biarticular Joint Torque에 가해지는 Disturbance Torque를 구함
        }
    }
    else
    {
        for (int i = 0; i < NDOF_LEG; i++)
            tauDist_hat[i] = 0;
            // RWDOB를 사용하지 않는 경우에는 Disturbance Torque를 계산하지 않고 0으로 리턴함
    }

    return tauDist_hat;
    // Disturbance Torque를 리턴함

}; // Rotating Workspace DOB

void controller::FOBRW(StateModel_* state_model, double cut_off) // Ground Reaction Force를 추정하기 위한 Controller
{
    cut_off_fob = 2*pi*cut_off;

    // Corioli & Gravity term 만들어 놓음 필요하면 쓰면 됩니닷

    rhs_fob = state_model->Lamda_nominal_FOB * state_model->qddot_bi_tustin;
    rhs_fob_old = state_model->Lamda_nominal_FOB * state_model->qddot_bi_tustin_old;
    // 본래 코드는 아래와 같았는데 이는 오류였음
    // rhs_fob = state_model->Lamda_nominal_FOB * state_model->qddot_bi_tustin_old;
    // rhs_fob_old = state_model->Lamda_nominal_FOB * state_model->qddot_bi_tustin;

    for (int i = 0; i < NDOF_LEG; i++)
    {
        lhs_fob_LPF[i] = lowpassfilter(state_model->tau_bi[i], state_model->tau_bi_old[i], lhs_fob_LPF_old[i], cut_off_fob);
        rhs_fob_LPF[i] = lowpassfilter(rhs_fob[i], rhs_fob_old[i], rhs_fob_LPF_old[i], cut_off_fob);

        tauExt_hat[i] = - (lhs_fob_LPF[i] - rhs_fob_LPF[i]);
        // 여기서 추정한 Ground Reaction Force 값은 System이 받는 External Force 값으로 Estimate한 것이기 때문에 Positive 값을 가진다
        // 그런데 실제로 이 Ground Reaction Force는 Robot Leg가 Compress되는 -r Direction으로 작용하는 Force이기 때문에 추정값에 -를 곱한다

    }
    forceExt_hat = state_model->jacbRW_trans_inv * tauExt_hat;
    printf("Estimated GRF: %f \n", forceExt_hat[0]);
    // Estimate한 Ground Reaction Force 값 출력


} // Rotating WorkspaceForce Observer
