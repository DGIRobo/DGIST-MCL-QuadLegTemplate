#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> //for bool
//#include<unistd.h> //for usl eep
#include <math.h>
//#include <resource.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "controller.h"
#include "dataLogging.h"
#include "animation.h"
#include "kinematics.h"
#include "trajectory.h"


mjvFigure figPosRW;         // RW position tracking plot
mjvFigure figFOB;           // RWFOB GRF estimation plot
mjvFigure figTrunkState;    // Trunk state plot

double simEndtime = 100;	// Simulation End Time
StateModel_  state_Model_FL;
controller ctrl_FL; // other class is in main loop
kinematics kin_FL;
trajectory tra_FL;


/***************** Main Controller *****************/
void mycontroller(const mjModel* m, mjData* d) // mj_step이 진행될 때 motor의 input을 이 mycontroller라는 함수를 먼저 실행해서 계산함.
{

    /* Controllers */
    int flag_DOB = 1;           // flag for switching ON/OFF RWDOB
    // RWDOB ON
    int flag_admitt = 1;        // flag for switching ON/OFF admittance control
    // admittance control ON
    double time_run = d->time;
    // 현재 time을 time_run에 저장

    //Admittance Control
    ctrl_FL.admittanceCtrl(&state_Model_FL,5,2,5000, flag_admitt); //parameter(omega_n=5,zeta=2,k=5000)
    // controller 헤더파일의 controller라는 class에서 admittanceCtrl이라는 함수를 실행

    // PID Control
    ctrl_FL.pid_gain_pos(200, 10, 150); //(kp,kd,freq)
    // controller 헤더파일의 controller라는 class에서 pid_gain_pos라는 함수를 실행
    state_Model_FL.tau_bi = state_Model_FL.jacbRW_trans * ctrl_FL.PID_pos(&state_Model_FL); // RW position feedback
    // globVariable이라는 헤더파일에서 state_Model_ 이라는 class의 tau_bi라는 변수에 position PID를 계산한 결과값을 저장함
    // 리턴받은 Position PID Control의 output(r 방향 힘, theta_r 방향 힘)에 RW coordinate 상에서 정의되는 Jacobian의 Transpose를 곱해서(Statics) Biarticular Torque값들을 계산

    // DOB control
    state_Model_FL.tau_bi = state_Model_FL.tau_bi + ctrl_FL.DOBRW(&state_Model_FL, 1000, flag_DOB);
    // globVariable이라는 헤더파일에서 state_Model_ 이라는 class의 tau_bi라는 변수에 RWDOB를 계산한 결과값을 더함
    // 리턴받은 RWDOB의 output(Biarticular Torque Compensation값)을 기존의 Biarticular Torque값들에 더함

    // Force Observer
    // (Algorithm 상으로 AdmittanceCtrl보다 앞에 있는데 여기에 위치해있는 이유?)
    // (아마도 가한 biarticular torque와 그에 따른 Ground Reaction Force를 짝을 맞추기 위해서인 것으로 추정)
    printf("-------------------GRF Comparing-------------------\n");
    printf("Real GRF: %f \n", d->sensordata[9]);
    // foot에 설치된 force sensor로부터 측정한 real ground reaction force 값을 가져와 출력
    ctrl_FL.FOBRW(&state_Model_FL, 100); // Rotating Workspace Force Observer (RWFOB)
    // controller라는 헤더파일에서 controller라는 class의 FOBRW라는 함수를 실행

   // Torque input Biarticular
    d->ctrl[0] = state_Model_FL.tau_bi[0] + state_Model_FL.tau_bi[1] ;
    // hip motor에 인가할 토크값을 biarticular motor 2개 토크값의 합으로 계산 (0번이 momo, 1번이 bi에 해당)
    d->ctrl[1] = state_Model_FL.tau_bi[1];
    // knee motor에 인가할 토크값을 biarticular motor 중 bi motor의 토크값으로 계산


    if (loop_index % data_frequency == 0) {
        save_data(m, d, &state_Model_FL);
    }
    loop_index += 1;
}


/***************** Main Function *****************/
int main(int argc, const char** argv)
{
    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if (argc < 2)
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);


    // make data
    d = mj_makeData(m);

    // Initialize GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {-88.95, -17.5, 1.8, 0.04, 0.000000, 0.27};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    fid = fopen(datapath, "w");
    init_save_data();

    // 여기서부터 본격적인 Control 알고리즘 시작
    // Initialization
    mju_copy(d->qpos, m->key_qpos + 0 * m->nq, m->nq);
    // xml의 key에 저장된 init position을 가져와서 simulation의 init position(qpos의 초기값)으로 설정.
    //d->qpos[1] = 0.546812;
    //d->qpos[2] = 2.59478;



    kin_FL.model_param_cal(m, d, &state_Model_FL); // state init is before. Caution Error.
    // kinematics 헤더파일에서 가져온 kinematics라는 class의 model_param_cal이라는 함수 실행
    // (현재 얻은 데이터들을 state_model에서 참조해서 Rotation Coordinate에서 정의되는 다양한 Model Predictive Control을 위한 Parameter들을 계산하는 것)
    kin_FL.state_init(m,d, &state_Model_FL);
    // kinematics 헤더파일에서 가져온 kinematics라는 classml state_init이라는 함수 실행
    // (state_model이라는 class가 저장하는 다양한 control을 위한 parameter들을 초기화하는 것)

    // custom controller
    mjcb_control = mycontroller;
    // main 파일에 저장된 컨트롤러 함수를 가져와서 mjcb_control로 저장.


    /***************** Simulation Loop *****************/
    // use the first while condition if you want to simulate for a period.
    int i = 0;
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        // Assuming MuJoCo can simulate faster than real-time, which it usually can,
        // this loop will finish on time for the next frame to be rendered at 60 fps.
        // Otherwise add a cpu timer and exit this loop when it is time to render.

        mjtNum simstart = d->time;
        // 시뮬레이션 시작시간을 simstart라는 변수로 기록
        //printf(" %f  %f \n", d->ctrl[0], d->ctrl[1]);
        state_Model_FL.time = d->time;
        // globVariable이라는 헤더파일에서 state_Model_ 이라는 class의 time이라는 변수에 현재 시간을 저장
        while (d->time - simstart < 1.0 / 60.0) // 1/60 sec마다 simulation이 1 step 진행됨
        {
            /* Trajectory Generation */
            int cmd_motion_type = 1; // control mode를 1(정지모드)로 설정, 0은 gait 모드를 의미
            int mode_admitt = 1;

            if (cmd_motion_type == 0)   // Squat, control mode를 0(gait 모드)로 설정한 경우
            {
                tra_FL.Squat(d->time, &state_Model_FL);
                // trajectory 헤더파일에서 가져온 trajectory라는 class의 Squat라는 함수를 실행
                // 현재 시간에 맞춘 trajectory 상의 point의 좌표를 state_Model_이라는 Class에 저장
            }
            else // control mode를 1(정지모드)로 설정한 경우
            {
                tra_FL.Hold(&state_Model_FL);  // Hold stance
                // trajectory 헤더파일에서 가져온 trajectory라는 class의 Hold라는 함수를 실행
                // 현재 시간에 맞춘 trajectory 상의 point의 좌표를 state_Model_ 이라는 Class에 저장
            }

            kin_FL.sensor_measure(m, d, &state_Model_FL); // get joint sensor data & calculate biarticular angles
            // kinematics 헤더파일의 kinematics라는 class에서 sensor_measure라는 함수를 실행
            // (simulation data d로부터 데이터를 받아 state_model에 저장하는 것)
            kin_FL.model_param_cal(m, d,&state_Model_FL); // calculate model parameters
            // kinematics 헤더파일의 kinematics라는 class에서 model_param_cal이라는 함수를 실행
            // (현재 얻은 데이터들을 state_model에서 참조해서 Rotation Coordinate에서 정의되는 다양한 Model Predictive Control을 위한 Parameter들을 계산하는 것)
            kin_FL.jacobianRW(&state_Model_FL);            // calculate RW Jacobian
            // kinematics 헤더파일의 kinematics라는 class에서 jacobianRW라는 함수를 실행
            // (현재 얻은 데이터들을 state_model에서 참조해서 Rotation Coordinate에서 정의되는 jacobian, jacobian_transpose, jacobian_transpose_inverse를 계산하는 것)

            if (d->time < 10) // simulation이 10초가 넘기 전까지는 다음의 항목을 cmd에 출력
            {
                //printf("ref: %f \n", state_Model_FL.posRW_ref[1]);
                //printf(" qddot_bi_tustin(0) = %f ,%f ", state_Model_FL.qddot_bi_tustin[0],state_Model_FL.qddot_bi_tustin[1]);
                //printf(" qddot_bi(1) = %f ,%f \n", state_Model_FL.qddot_bi[0],state_Model_FL.qddot_bi[1]);

            }


            kin_FL.fwdKinematics_cal(&state_Model_FL);     // calculate RW Kinematics
            // kinematics 헤더파일의 kinematics라는 class에서 fwdkinematics_cal이라는 함수를 실행

            mj_step(m, d);
            // mujoco simulation을 1 step 진행 (이때 controller 계산도 진행함)

            kin_FL.state_update(&state_Model_FL);
            // kinematics라는 헤더파일의 kinematics라는 class에서 state_update라는 함수를 실행
            ctrl_FL.ctrl_update();
            // controller하는 헤더파일의 controller라는 class에서 ctrl_update라는 함수를 실행
        }
        // 여기까지가 주요 Control 코드 끝

        if (d->time >= simEndtime) { // simulation이 사전에 정한 simEndtime을 넘게 되면 simulation을 종료함
            fclose(fid);
            break;
        }
        //printf("%f \n", state_Model_FL.deltaPos[0]);
        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);




        // update scene and render
        //opt.frame = mjFRAME_WORLD;
        //cam.lookat[0] = d->qpos[0];
        //cam.lookat[1] = 0;
        //cam.lookat[2] = 0;
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}
