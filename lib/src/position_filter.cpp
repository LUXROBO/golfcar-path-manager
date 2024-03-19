#include "position_filter.h"
#include "model_matrix_double.h"
#include <cmath>

static double H_yaw_data[4] = {0,0,0,1};
static ModelMatrix_D H_yaw(2, 2, H_yaw_data);

static double H_xy_data[4] = {1,0,0,0};
static ModelMatrix_D H_xy(2, 2, H_xy_data);

static double H_pas_data[4] = {1,0,0,0};
static ModelMatrix_D H_pos(2, 2, H_pas_data);

static double R_imu_data[4] = {0,0,0,0};
static ModelMatrix_D R_imu(2, 2, R_imu_data);

static double R_xy_mask_data[4] = {1, 0,
                                     0, 1};
static ModelMatrix_D R_xy_mask(2, 2, R_xy_mask_data);

static double R_yaw_mask_data[4] = {0, 0,
                                      0, 0,};
static ModelMatrix_D R_yaw_mask(2, 2, R_yaw_mask_data);

// const double dt = 0.01;
const double W = 2.18;

/*
GPS Quality indicator:
0: Fix not valid
1: GPS fix
2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
3: Not applicable
4: RTK Fixed, xFill
5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
6: INS Dead reckoning
*/
static double R_gps_quality1_data[4] = {1,0,
                                        0,1};   /** gps fix*/
static double R_gps_quality2_data[4] = {0.8,0.5,
                                        0.5,1.4};   /** gps differintial fix*/
static double R_gps_quality4_data[4] = {0.00002   ,0.000002,
                                          0.000002  ,0.00005};   /** rtk fix*/
static double R_gps_quality5_data[4] = {0.00168   , -0.00231,
                                          -0.00231  ,0.00332};   /** rtk float*/

static ModelMatrix_D R_gps_xy[6] = {
    ModelMatrix_D(2, 2),
    ModelMatrix_D(2, 2, R_gps_quality1_data),
    ModelMatrix_D(2, 2, R_gps_quality2_data),
    ModelMatrix_D(2, 2),
    ModelMatrix_D(2, 2, R_gps_quality4_data),
    ModelMatrix_D(2, 2, R_gps_quality5_data),
};

/*
position = [x, y, yaw]

지금 필요한 것
 - 각 갱신 조건에 맞는 H matrix
 - P matrix의 초기값
 - Q matrix 값 설정
 - R_gps matrix 값 설정
  - 아마 gps quality에 따라 설정 필요
 - R_imu matrix 값 설정
 - dt 입력 고려
*/
position_filter::position_filter()
{
    this->x = ModelMatrix_D::zero(2, 1);
    this->x_predict = this->x;
    this->P = ModelMatrix_D::identity(2, 2);
    this->P_predict = this->P;
    this->Q = ModelMatrix_D::zero(2, 2);
    this->Q.set(0, 0, 0.01);
    this->Q.set(1, 1, 0.01);
    this->R = ModelMatrix_D::identity(2, 2);
    this->H = ModelMatrix_D::identity(this->x.row(), this->x.row());
    this->K = ModelMatrix_D::identity(this->x.row(), this->x.row());
    this->init_flag = false;

    this->yaw_H = 1;
    this->yaw_P = 0;
    this->yaw_K = 0;
    this->yaw = 0;
    this->yaw_Q = 0.0001;
    this->yaw_R = 0.01;
}

position_filter::~position_filter()
{

}

position_filter::position_filter(ModelMatrix_D x, ModelMatrix_D P)
{
    this->x = x;
    this->x_predict = x;
    this->P = P;
    this->P_predict = P;
    this->Q = ModelMatrix_D::identity(this->x.row(), this->x.row());
    this->R = ModelMatrix_D::identity(this->x.row(), this->x.row());

    this->H = ModelMatrix_D::identity(this->x.row(), this->x.row());
    this->K = ModelMatrix_D::identity(this->x.row(), this->x.row());

    this->init_flag = false;
}

ModelMatrix_D position_filter::state_equation_jacobi(ModelMatrix_D x0, ModelMatrix_D input)
{
    /*
    x = [x,             input = [v,
         y,                      steer(slip_angle)]
         yaw]

    x = x + v * dt * cos(yaw + steer)
    y = y + v * dt * sin(yaw + steer)
    yaw = yaw + v * dt * cos(steer) * tan(steer) / W(wheel base);

    jacobian state = [x, y, yaw, v, steer]

    jacobian matrix =
    [
        1,      0,      -v * dt * sin(yaw + steer), dt * cos(yaw + steer),                      -v * dt * sin(yaw + steer),
        0,      1,      v * dt * cos(yaw + steer),  dt * sin(yaw + steer),                      v * dt * cos(yaw + steer),
        0,      0,      1                           dt * cos(steer) * tan(steer) / W,           -v * dt * sin(steer) * tan(steer) / W + v * dt / cos(steer) / W
        0,      0,      0,                          1,                                          0,
        0,      0,      0,                          0,                                          1,
    ]
    */
    // ModelMatrix_D jacobian = ModelMatrix_D::zero(5, 5);
    ModelMatrix_D jacobian = ModelMatrix_D::zero(x0.row(), x0.row());
    double v = input.get(0, 0);
    double yaw = input.get(2, 0);
    double steer = input.get(1, 0);
    double yaw_steer = path_tracker::pi_to_pi(yaw + steer);
    double dt = input.get(3, 0);

    jacobian.set(0, 0, 1.0);
    jacobian.set(0, 2, -1 * v * dt * std::sin(yaw_steer));
    // jacobian.set(0, 3, dt * (std::cos(yaw_steer)));
    // jacobian.set(0, 4, -1 * input.get(0, 0) * dt * std::sin(yaw_steer));

    jacobian.set(1, 1, 1.0);
    jacobian.set(1, 2, v * dt * std::cos(yaw_steer));
    // jacobian.set(1, 3, dt * (std::sin(yaw_steer)));
    // jacobian.set(1, 4, input.get(0, 0) * dt * std::cos(yaw_steer));

    // jacobian.set(2, 2, 1);
    // jacobian.set(2, 3, dt * std::cos(steer) * tan(steer) / W);
    // jacobian.set(2, 3, -1 * input.get(0, 0) * dt * std::sin(steer) * tan(steer) / W
    //                    + input.get(0, 0) * dt / std::cos(steer) / W);

    // jacobian.set(3, 3, 1);

    // jacobian.set(4, 4, 1);

    return jacobian;
}

// ModelMatrix_D position_filter::predict_xy(ModelMatrix_D x0, ModelMatrix_D input)
// {
//     /*
//         x = [x,             input = [v,
//             y,                      steer(slip_angle)]
//             yaw]

//         x = x + v * dt * cos(yaw + steer)
//         y = y + v * dt * sin(yaw + steer)
//         yaw = yaw + v * dt * cos(steer) * tan(steer) / W(wheel base);
//     */
//     ModelMatrix_D x1 = ModelMatrix_D::zero(3, 1);
//     // yaw + steer
//     double yaw_steer = (x0.get(2, 0) + input.get(1, 0));
//     double steer = input.get(1, 0);
//     double dt = input.get(2, 0);
//     double v = input.get(0, 0);

//     x1.set(0, 0, x0.get(0, 0) + v * dt * std::cos(x0.get(2, 0)));
//     x1.set(1, 0, x0.get(1, 0) + v * dt * std::sin(x0.get(2, 0)));

//     return x1;
// }


ModelMatrix_D position_filter::predict_xy(ModelMatrix_D input)
{
    if (this->init_flag) {
        ModelMatrix_D A = this->state_equation_jacobi(this->x, input);
        double v = input.get(0, 0);
        double steer = input.get(1, 0);
        double yaw = input.get(2, 0);
        double yaw_steer = (yaw + steer);
        double dt = input.get(3, 0);

        this->x.set(0, 0, this->x.get(0, 0) + v * dt * std::cos(yaw_steer));
        this->x.set(1, 0, this->x.get(1, 0) + v * dt * std::sin(yaw_steer));
        this->P = A * this->P * A.transpose() + this->Q;
    }
    return this->x;
}

ModelMatrix_D position_filter::estimate_yaw_with_imu(ModelMatrix_D z)
{
    // this->H = H_yaw;
    this->R = R_imu;    // imu error 공분산
    return this->estimate(z);
}

ModelMatrix_D position_filter::estimate_xy_with_gps(ModelMatrix_D z, int quality)
{
    // this->H = H_xy;
    // std::cout << "estimate xy" << std::endl;
    this->R = R_gps_xy[quality];
    // this->R = ModelMatrix_D::identity(this->x.row(), this->x.row());

    return this->estimate(z);
}

ModelMatrix_D position_filter::estimate(ModelMatrix_D z)
{
    if (this->init_flag) {
        this->K = this->P * H.transpose() * (((this->H * this->P * this->H.transpose()) + this->R).inverse());
        this->x = this->x + this->K * (z - this->H * this->x);
        this->P = this->P - this->K * this->H * this->P;

        // std::cout << "P" << std::endl;
        // for (int i=0; i<2; i++) {
        //     for (int j=0; j<2; j++) {
        //         std::cout << this->P.get(i, j) << "\t";
        //     }
        //     std::cout << "\n";
        // }
        // std::cout << "\n";
        // std::cout << "K" << std::endl;
        // for (int i=0; i<2; i++) {
        //     for (int j=0; j<2; j++) {
        //         std::cout << this->K.get(i, j) << "\t";
        //     }
        //     std::cout << "\n";
        // }
        // std::cout << "\n";
    }
    return this->x;
}

double position_filter::predict_yaw(double input, double dt)
{
    this->yaw = path_tracker::pi_to_pi(this->yaw + input * dt);
    this->yaw_P = this->yaw_P + this->yaw_Q;
}

double position_filter::estimate_yaw(double z)
{
    this->yaw_K = this->yaw_P / (this->yaw_P + this->yaw_R);
    this->yaw = this->yaw + this->yaw_K * (z - this->yaw);
    this->yaw_P = this->yaw_P - this->yaw_K * this->yaw_P;

    return this->yaw;
}