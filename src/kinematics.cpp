#include "kinematics.h"

kinematics::kinematics(){};

kinematics::~kinematics(){};

void kinematics::state_update(StateModel_* state_model)
{

    //printf("%f, %f %f \n\n", state_model->forceExt_hat[0], state_model->forceExt_hat_old[0], state_model->forceExt_hat_old2[0]);
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Joint
        state_model->q_old[i] = state_model->q[i];
        state_model->q_bi_old[i] = state_model->q_bi[i]; //okay
        state_model->qdot_bi_old[i] = state_model->qdot_bi[i];
        state_model->qdot_bi_tustin_old[i] = state_model->qdot_bi_tustin[i]; //okay
        state_model->qddot_bi_tustin_old[i] = state_model->qddot_bi_tustin[i]; //okay

        // Feedback - RW Kinematics
        state_model->posRW_old[i] = state_model->posRW[i];
        state_model->velRW_old[i] = state_model->velRW[i];
        state_model->posRW_ref_old2[i] = state_model->posRW_ref_old[i];
        state_model->posRW_ref_old[i] = state_model->posRW_ref[i];
        state_model->velRW_ref_old[i] = state_model->velRW_ref[i];

        // control input
        state_model->ctrl_input_RW_old[i] = state_model->ctrl_input_RW[i];
        state_model->tau_bi_old[i] = state_model->tau_bi[i];
    }

    state_model->H_old = state_model->H;
    //printf("%f, %f \n", state_model->lhs_fob_LPF[0], state_model->lhs_fob_LPF_old[0]);
};


void kinematics::model_param_cal(const mjModel* m, mjData* d, StateModel_* state_model)
{
    cut_off_cal = 1/(2*pi*150);

    /* Trunk Parameters */
    m_hip = 2.5;
    m_trunk_front = 0.;
    m_trunk_rear = 0.;
    m_trunk = 4 * m_hip + m_trunk_front + m_trunk_rear;

    /* Leg Parameters */
    L = 0.25;
    d_thigh = 0.11017; // local position of CoM of thigh
    d_shank = 0.12997; // local position of CoM of shank
    // printf("d_thigh : %f, d_shank : %f \n", d_thigh, d_shank);

    m_thigh = 1.017; // mass of thigh link
    m_shank = 0.143; // mass of shank link
    m_leg = m_thigh + m_shank;
    m_total = m_trunk + 4 * m_leg;
    // printf("m_thigh : %f, m_shank : %f \n", m_thigh, m_shank);

    Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
    Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM
    // printf("Izz_thigh : %f, Izz_shank : %f \n", Izz_thigh, Izz_shank);

    Jzz_thigh =
        Izz_thigh + m_thigh * pow(d_thigh, 2); // MoI of thigh w.r.t. HFE
    Jzz_shank =
        Izz_shank + m_shank * pow(d_shank, 2); // MoI of thigh w.r.t. KFE
    // printf("Jzz_thigh : %f, Jzz_shank : %f \n", Jzz_thigh, Jzz_shank);

    double M1 = Jzz_thigh + m_shank * pow(L, 2);
    double M2 = m_shank * d_shank * L * cos(state_model->q[1]);
    double M12 = Jzz_shank;

    MatInertia_bi(0,0) = M1;
    MatInertia_bi(0,1) = M12;
    MatInertia_bi(1,0) = M12;
    MatInertia_bi(1,1) = M2;
    // RW Coodinate에서 사용하는 Mass Matrix 계산 완료

    state_model->Lamda_nominal_FOB(0,0) = M1;
    state_model->Lamda_nominal_FOB(0,1) = M12;
    state_model->Lamda_nominal_FOB(1,0) = M12;
    state_model->Lamda_nominal_FOB(1,1) = M2;
    // Mass Matrix를 FOB에서 사용하기 위해서 state_model로 따로 빼서 저장


    JzzR_thigh  = Jzz_thigh + Jzz_shank + m_shank * pow(L, 2) - 2 * m_shank * d_shank * L * cos(state_model->q[1]);
    JzzR_couple = Jzz_thigh + m_shank * pow(L, 2) - Jzz_shank;
    JzzR_shank = Jzz_thigh + Jzz_shank+ m_shank * pow(L, 2) + 2 * m_shank * d_shank * L * cos(state_model->q[1]);
    // printf("JzzR_thigh : %f, JzzR_shank : %f, JzzR_couple : %f \n", JzzR_thigh, JzzR_shank,
    // JzzR_couple);

    MatInertia_RW(0,0) = JzzR_thigh / (4 * pow(L, 2) * pow(sin(state_model->q[1] / 2), 2));
    MatInertia_RW(0,1) = JzzR_couple / (2 * pow(L, 2) * sin(state_model->q[1]));
    MatInertia_RW(1,0) = JzzR_couple / (2 * pow(L, 2) * sin(state_model->q[1]));
    MatInertia_RW(1,1) = JzzR_shank / (4 * pow(L, 2) * pow(cos(state_model->q[1] / 2), 2));
    // torque 관계식으로부터 나오는 Mass Matrix를 Jacobian을 사용해서 Force 관계식으로 바꾸면 나오는 Inertia Matrix를 계산

    Inertia_DOB(0,0) = MatInertia_RW(0,0);
    Inertia_DOB(0,1) = 0;
    Inertia_DOB(1,0) = 0;
    Inertia_DOB(1,1) = MatInertia_RW(1,1);
    // Inertia Matrix를 토대로 DOB에서 사용하기 위한 Inertia Matrix를 계산

    double check[4] = { 0 };

    state_model->Lamda_nominal_DOB = state_model->jacbRW_trans*Inertia_DOB*state_model->jacbRW;
    // DOB에서 활용하기 위한 Inertia Matrix를 RW Coodinate에서 사용하기 위한 RW Inertia Matrix로 변환하고 DOB에서 사용하기 위해서 state_model로 따로 빼서 저장

    //Coriolis & Gravity Force 계산
    H[0] = -m_shank * d_shank * L * sin(state_model->q[1]) * pow(state_model->qdot_bi[1], 2)
         - g * (m_thigh * d_thigh + m_shank * L) * cos(state_model->q_bi[0]);

    H[1] = m_shank * d_shank * L * sin(state_model->q[1]) * pow(state_model->qdot_bi[0], 2)
         - g * m_shank * d_shank * cos(state_model->q_bi[1]);

}; // param_model parameter

void kinematics::sensor_measure(const mjModel* m, mjData* d, StateModel_* state_model)
{
    cut_off_cal = 1/(2*pi*150);
    // Q filter의 cut-off frequency f 설정
    /*** (Serial) Joint position ***/
    state_model->q[0] = d->qpos[1];  // (relative) HFE angle
    state_model->q[1] = d->qpos[2];  // (relative) KFE angle
    // simulation으로부터 serial joint position값들을 받아와서 state_model의 q로 저장

    // state_model->q[0] = d->sensordata[6]; // (relative) HFE angle
    // state_model->q[1] = d->sensordata[7]; // (relative) KFE angle
    // sensor로부터 serial joint position값들을 받아와서 state_model의 q로 저장
    state_model->touch_sensor = d->sensordata[8];
    // foot에 설치된 touch sensor로부터 touch sensor값을 받아와서 state model의 touch_sensor값으로 저장

    /*** Biarticular Transformation ***/
    state_model->q_bi[0] = d->qpos[1];              // (absolute) HFE angle
    state_model->q_bi[1] = d->qpos[1] + d->qpos[2]; // (absolute) KFE angle
    // simulation으로부터 serial joint position값들을 받아와서 biarticular joint position값들로 변환

    // state_model->q_bi[0] = d->sensordata[6];                    // (absolute) HFE angle
    // state_model->q_bi[1] = d->sensordata[6] + d->sensordata[7]; // (absolute) KFE angle

    // printf("q1 : %f, q2 : %f \n", d->qpos[0], d->qpos[1]);
    // printf("qm : %f, qb : %f \n\n", q_bi[0], q_bi[1]);

    state_model->qdot_bi[0] = d->qvel[1];
    state_model->qdot_bi[1] = d->qvel[1] + d->qvel[2];
    // simulation으로부터 serial joint velocity값들을 받아와서 biarticular joint velocity값들로 변환
    // printf("qd1 : %f, qd2 : %f \n", d->qvel[0], d->qvel[1]);
    // printf("qdm : %f, qdb : %f \n\n", qdot_bi[0], qdot_bi[1]);

    state_model->qddot_bi[0] = d->qacc[1];
    state_model->qddot_bi[1] = d->qacc[1] + d->qacc[2];
    // simulation으로부터 serial joint acceleration값들을 받아와서 biarticular joint accelertation값들로 변환
    // printf("qdd1 : %f, qdd2 : %f \n", d->qacc[0], d->qacc[1]);S
    // printf("qddm : %f, qddb : %f \n\n", qddot_bi[0], qddot_bi[1]);

    for (int i = 0; i < NDOF_LEG; i++)
    {
        state_model->qdot_bi_tustin[i] =
            tustin_derivative(state_model->q_bi[i], state_model->q_bi_old[i], state_model->qdot_bi_tustin_old[i],
                cut_off_cal);
        // filter 헤더파일에서 tustin_derivative 함수를 실행해서 나온 결과값을 tustin 변환한 biarticular joint의 velocity값 qdot_bi_tustin으로 저장
        state_model->qddot_bi_tustin[i] =
            tustin_derivative(state_model->qdot_bi_tustin[i], state_model->qdot_bi_tustin_old[i],
                state_model->qddot_bi_tustin_old[i], cut_off_cal);
        // filter 헤더파일에서 tustin_derivative 함수를 실행해서 나온 결과값을 tustin 변환한 biarticular joint의 acceleration값 qddot_bi_tustin으로 저장
    }
    // printf("qdot_bi[0]: %f, qdot_bi[1]: %f \n qdot_bi_tust[0]: %f, qdot_bi_tust[1]: %f \n\n",
    // state_Model_FL.qdot_bi[0],
    //        state_Model_FL.qdot_bi[1], state_Model_FL.qdot_bi_tustin[0], state_Model_FL.qdot_bi_tustin[1]);
    // printf("qddot_bi[0]: %f, qddot_bi[1]: %f \n qddot_bi_tust[0]: %f, qddot_bi_tust[1]: %f \n\n",
    //        state_Model_FL.qddot_bi[0], state_Model_FL.qddot_bi[1], state_Model_FL.qddot_bi_tustin[0],
    //        state_Model_FL.qddot_bi_tustin[1]);
};

void kinematics::jacobianRW(StateModel_* state_model)
{
  // 현재 state_model class에 있는 값들을 사용해서 Rotation Coordinate에서 정의된 Jacobian, Jacobian의 Transpose, Jacobian의 Transpose의 Inverse를 계산 및 저장하는 함수
    /*** Rotating Workspace ***/
    state_model->jacbRW(0,0) =  L * sin(state_model->q[1] / 2);
    state_model->jacbRW(0,1) = -L * sin(state_model->q[1] / 2);
    state_model->jacbRW(1,0) =  L * cos(state_model->q[1] / 2);
    state_model->jacbRW(1,1) =  L * cos(state_model->q[1] / 2);
    // Rotation Work Space Coordinate에서 정의되는 Jacobian을 계산해서 state_model class의 jacobRW 변수에 저장

    state_model->jacbRW_trans = state_model->jacbRW.transpose();
    // 앞서서 구한 jacobRW를 transpose 시켜서 state_model class의 jacobRW_trans 변수에 저장

    state_model->jacbRW_trans_inv = state_model->jacbRW_trans.inverse();
    // 앞서서 구한 jacobRW_trans를 inverse 시켜서 state_model_class의 jacobRW_trans_inv 변수에 저장
};

void kinematics::fwdKinematics_cal(StateModel_* state_model)
{
    state_model->posRW[0] = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2); // r
    // state_model라는 class에서 biarticular joint angle q_m, q_b를 참조해서 RW coordinate에서의 r을 계산 및 저장
    state_model->posRW[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;                           // qr
    // state_model라는 class에서 biarticular joint angle q_m, q_b를 참조해서 RW coordinate에서의 theta_r을 계산 및 저장

    state_model->velRW = state_model->jacbRW*state_model->qdot_bi_tustin;
    // state_model라는 class에서 tustin 변환을 한 biarticular joint velocity와 RW coordinate에서의 jacobian을 참조해서 RW coordinate에서의 rdot과 theta_rdot을 계산 및 저장
};

void kinematics::state_init(const mjModel* m, mjData* d, StateModel_* state_model)
{
    state_model->q[0] = d->qpos[1];
    state_model->q[1] = d->qpos[2];
    // simulation data d에 저장된 초기 각 joint의 qpos값들을 받아와서 state_model라는 class가 가지고 있는 q 변수들에 저장

    state_model->q_bi[0] = d->qpos[1];
    state_model->q_bi[1] = d->qpos[1] + d->qpos[2];
    // simulation data d에 저장된 초기 각 joint의 qpos값들을 받아와서 biarticular joint qm, qb의 값을 계산하고 state_model라는 class가 가지고 있는 q_bi 변수에 저장

    // RW coordinates initialization
    state_model->r0 = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2);
    // RW coordinate에서 초기 r값 r0를 계산해서 state_model라는 class가 가지고 있는 r0 변수에 저장

    state_model->posRW[0] = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2);
    // RW coordinate에서 r값을 계산해서 state_model라는 class가 가지고 있는 posRW[0] 변수에 저장
    state_model->posRW[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;
    // RW coordinate에서 theta_r값을 계산해서 state_model라는 class가 가지고 있는 posRW[1] 변수에 저장

    state_model->posRW_ref[0] = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2);
    // 현재 계산한 r값을 그대로 desired r값으로 설정함.
    state_model->posRW_ref[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;
    // 현재 계산한 theta_r값을 그대로 desired theta_r값으로 설정함


    state_model->touch_sensor = 0;
    // state_model라는 class가 가지고 있는 touch sensor 변수를 0으로 초기화함

    state_model->Lamda_nominal_DOB(0,0)= 0.0;
    state_model->Lamda_nominal_DOB(0,1)= 0.0;
    state_model->Lamda_nominal_DOB(1,0)= 0.0;
    state_model->Lamda_nominal_DOB(1,1)= 0.0;
    // state_model라는 class가 가지고 있는 DOB에서 사용하기 위한 RW Inertia Matrix를 모두 초기값 0으로 초기화함

    //printf("%f \n", state_model->r0);
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Joint coordinates [k-1] values
        // biarticular의 joint들의 Discrete Position값들을 초기화함
        state_model->q_bi_old[i] = state_model->q_bi[i];
        // state_model라는 class가 가지고 있는 현재 biarticular의 joint angle들 q_bi를 state_model라는 class가 가지고 있는 과거 biarticular의 joint angle들 q_bi_old로 저장

        // biarticular의 joint들의 Discrete Velocity값들을 초기화함
        state_model->qdot_bi[i] = 0.0;
        // biarticular의 joint들의 velocity들 qdot_bi를 0으로 초기화
        state_model->qdot_bi_tustin[i] = 0.;
        // biarticular의 joint들의 현재 velocity들의 tustin 변환을 한 값 qdot_bi_tustin을 0으로 초기화함
        state_model->qdot_bi_tustin_old[i] = state_model->qdot_bi_tustin[i];
        // biarticular의 joint들의 과거 velocity들의 tustin 변환을 한 값 qdot_bi_tustin_old를 현재 velocity들의 tustin 변환을 한 값 qdot_bi_tustin으로 초기화함

        // biarticular의 joint들의 Discrete Acceleration값들을 초기화함
        state_model->qddot_bi[i] = 0.;
        // biarticular의 joint들의 acceleration들 qddot_bi를 0으로 초기화
        state_model->qddot_bi_tustin[i] = 0.;
        // biarticular의 joint들의 acceleration들의 tustin 변환을 한 값 qddot_bi_tustin을 0으로 초기화함
        state_model->qddot_bi_tustin_old[i] = state_model->qddot_bi_tustin[i];
        // biarticular의 joint들의 과거 acceleration들의 tustin 변환을 한 값 qddot_bi_tustin_old를 현재 acceleration들의 tustin 변환을 한 값 qddot_bi_tustin으로 초기화

        // biarticular의 joint들의 Discrete Torque값들을 초기화함
        state_model->tau_bi[i] = 0.;
        // biarticular의 joint들의 현재 torque값 tau_bi를 0으로 초기화함
        state_model->tau_bi_old[i] = state_model->tau_bi[i];
        // biarticular의 joint들의 과거 torque값 tau_bi_old를 현재 torque값 tau_bi로 초기화

        // RW coordinates [k-1] values
        // RW coordinate r, theta_r 값들을 초기화함.
        state_model->posRW_old[i] = state_model->posRW[i];
        // RW coordinate의 과거 r, theta_r 값을 현재 r, theta_r값으로 초기화
        state_model->posRW_ref_old[i] = state_model->posRW_ref[i];
        // RW coodinate의 과거 reference r, theta_r값을 현재 reference r, theta_r값으로 초기화
        state_model->posRW_ref_old2[i] = state_model->posRW_ref_old[i];
        // RW coordinate의 과거의 과거 reference r, theta_r값을 과거 reference r, theta_r값으로 초기화

        // RW coordinate의 rdot, theta_rdot 값들을 초기화
        state_model->velRW[i] = .0;
        // RW coordinate의 현재 rdot, theta_rdot값을 0으로 초기화
        state_model->velRW_old[i] = state_model->velRW[i];
        // RW coordinate의 과거 rdot, theta_rdot값을 현재 rdot, theta_rdot값으로 초기화
        state_model->velRW_ref[i] = 0.;
        // RW coordinate의 현재 reference rdot, theta_rdot값을 0으로 초기화
        state_model->velRW_ref_old[i] = state_model->velRW_ref[i];
        // RW coordinate의 과거 reference rdot, theta_rdot값을 현재 rdot, theta_rdot값으로 초기화

        // RW coordinate의 r과 theta_r에 관한 control input값을 0으로 초기화
        state_model->ctrl_input_RW[i] = 0.;
        // RW coordinate의 r과 theta_r에 관한 과거 control input값을 현재 control input값으로 초기화
        state_model->ctrl_input_RW_old[i] = state_model->ctrl_input_RW[i];


        // Mg Trajectory
        // tau_ff를 0으로 초기화
        state_model->tau_ff[i]=0.;
    }
    // printf("%f, %f \n", state_model->q_bi_old[0], state_model->q_bi_old[1]);
};
