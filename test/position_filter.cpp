#include "position_filter.h"
#include "model_matrix_double.h"
#include <cmath>

static double H_yaw_data[9] = {0,0,0,0,0,0,0,0,1};
static ModelMatrix H_yaw(3, 3, H_yaw_data);

static double H_xy_data[9] = {1,0,0,0,1,0,0,0,0};
static ModelMatrix H_xy(3, 3, H_xy_data);

static double H_pas_data[9] = {1,0,0,0,1,0,0,0,1};
static ModelMatrix H_pos(3, 3, H_pas_data);

static double R_imu_data[9] = {0,0,0,0,0,0,0,0,0};
static ModelMatrix R_imu(3, 3, R_imu_data);

static double R_xy_mask_data[9] = {1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 0};
static ModelMatrix R_xy_mask(3, 3, R_xy_mask_data);

static double R_yaw_mask_data[9] = {0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 1};
static ModelMatrix R_yaw_mask(3, 3, R_yaw_mask_data);

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
static double R_gps_quality1_data[9] = {1,0,0,0,1,0,0,0,0};   /** gps fix*/
static double R_gps_quality2_data[9] = {1,0,0,0,1,0,0,0,0};   /** gps differintial fix*/
static double R_gps_quality4_data[9] = {0.00002   ,0.000002   ,0,
                                          0.000002  ,0.00005    ,0,
                                          0         ,0          ,0.01};   /** rtk fix*/
static double R_gps_quality5_data[9] = {0.00168   , -0.00231  ,0,
                                          -0.00231  ,0.00332    ,0,
                                          0         ,0          ,0.01};   /** rtk float*/

static ModelMatrix R_gps_xy[6] = {
    ModelMatrix(3, 3),
    ModelMatrix(3, 3, R_gps_quality1_data),
    ModelMatrix(3, 3, R_gps_quality2_data),
    ModelMatrix(3, 3),
    ModelMatrix(3, 3, R_gps_quality4_data),
    ModelMatrix(3, 3, R_gps_quality5_data),
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
    this->x = ModelMatrix::zero(3, 1);
    this->x_predict = this->x;
    this->P = ModelMatrix::identity(3, 3);
    this->P_predict = this->P;
    this->Q = ModelMatrix::zero(3, 3);
    this->Q.set(0, 0, 0.1);
    this->Q.set(1, 1, 0.1);
    this->Q.set(2, 2, 0.1);

    this->R = ModelMatrix::identity(3, 3);

    this->H = ModelMatrix::identity(this->x.row(), this->x.row());
    this->K = ModelMatrix::identity(this->x.row(), this->x.row());
    this->init_flag = false;
}

position_filter::~position_filter()
{

}

position_filter::position_filter(ModelMatrix x, ModelMatrix P)
{
    this->x = x;
    this->x_predict = x;
    this->P = P;
    this->P_predict = P;
    this->Q = ModelMatrix::identity(this->x.row(), this->x.row());
    this->R = ModelMatrix::identity(this->x.row(), this->x.row());

    this->H = ModelMatrix::identity(this->x.row(), this->x.row());
    this->K = ModelMatrix::identity(this->x.row(), this->x.row());

    this->init_flag = false;
}

ModelMatrix position_filter::state_equation_jacobi(ModelMatrix x0, ModelMatrix input)
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
    // ModelMatrix jacobian = ModelMatrix::zero(5, 5);
    ModelMatrix jacobian = ModelMatrix::zero(x0.row(), x0.row());
    double yaw_steer = (x0.get(2, 0) + input.get(1, 0));
    double steer = input.get(1, 0);
    double dt = input.get(2,0); 

    jacobian.set(0, 0, 1.0);
    jacobian.set(0, 2, -1 * input.get(0, 0) * dt * std::sin(yaw_steer));
    // jacobian.set(0, 3, dt * (std::cos(yaw_steer)));
    // jacobian.set(0, 4, -1 * input.get(0, 0) * dt * std::sin(yaw_steer));

    jacobian.set(1, 1, 1.0);
    jacobian.set(1, 2, input.get(0, 0) * dt * std::cos(yaw_steer));
    // jacobian.set(1, 3, dt * (std::sin(yaw_steer)));
    // jacobian.set(1, 4, input.get(0, 0) * dt * std::cos(yaw_steer));

    jacobian.set(2, 2, 1);
    // jacobian.set(2, 3, dt * std::cos(steer) * tan(steer) / W);
    // jacobian.set(2, 3, -1 * input.get(0, 0) * dt * std::sin(steer) * tan(steer) / W
    //                    + input.get(0, 0) * dt / std::cos(steer) / W);

    // jacobian.set(3, 3, 1);

    // jacobian.set(4, 4, 1);

    return jacobian;
}

ModelMatrix position_filter::predict_position(ModelMatrix x0, ModelMatrix input)
{
    /*
        x = [x,             input = [v,
            y,                      steer(slip_angle)]
            yaw]

        x = x + v * dt * cos(yaw + steer)
        y = y + v * dt * sin(yaw + steer)
        yaw = yaw + v * dt * cos(steer) * tan(steer) / W(wheel base);
    */
    ModelMatrix x1 = ModelMatrix::zero(3, 1);
    // yaw + steer
    double yaw_steer = (x0.get(2, 0) + input.get(1, 0));
    double steer = input.get(1, 0);
    double dt = input.get(2, 0);

    // x(k+1) = x(k) + v * dt * cos(yaw + steer);
    x1.set(0, 0, x0.get(0, 0) + input.get(0, 0) * dt * cos(x0.get(2, 0)));
    // x(k+1) = x(k) + v * dt * sin(yaw + steer);
    x1.set(1, 0, x0.get(1, 0) + input.get(0, 0) * dt * sin(x0.get(2, 0)));
    // x1.set(2, 0, x0.get(2, 0) + input.get(0, 0) * dt * cos(steer) * tan(steer) / W);
    // x1.set(2, 0, x0.get(2, 0) + input.get(0, 0) * dt * sin(steer) / W);

    x1.set(2, 0, path_tracker::pi_to_pi(x0.get(2, 0) + input.get(0, 0) * dt * tan(steer) / W));

    return x1;
}

ModelMatrix position_filter::predict(ModelMatrix input)
{
    if (this->init_flag) {
        ModelMatrix A = this->state_equation_jacobi(this->x, input);
        this->x = predict_position(this->x, input);
        this->P = A * this->P * A.transpose() + this->Q;
    }
    return this->x;
}

ModelMatrix position_filter::estimate_yaw_with_imu(ModelMatrix z)
{
    // this->H = H_yaw;
    this->R = R_imu;    // imu error 공분산
    return this->estimate(z);
}

ModelMatrix position_filter::estimate_xy_with_gps(ModelMatrix z, int quality)
{
    // this->H = H_xy;
    // std::cout << "estimate xy" << std::endl;
    this->R = R_gps_xy[quality] * R_xy_mask;
    // this->R = ModelMatrix::identity(this->x.row(), this->x.row());
    
    return this->estimate(z);
}

ModelMatrix position_filter::estimate_yaw_with_gps(ModelMatrix z, int quality)
{
    // this->H = H_xy;
    // std::cout << "estimate yaw" << std::endl;
    this->R = R_gps_xy[quality] * R_yaw_mask;
    return this->estimate(z);
}

ModelMatrix position_filter::estimate(ModelMatrix z)
{   
    if (this->init_flag) {
        this->K = this->P * (H.transpose() * this->H * this->P * this->H.transpose() + this->R).inverse();
        this->x = this->x + this->K * (z - this->H * this->x);
        this->P = this->P - this->K * this->H * this->P;

        std::cout << "P" << std::endl;
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                std::cout << this->P.get(i, j) << "\t";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
        std::cout << "K" << std::endl;
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                std::cout << this->K.get(i, j) << "\t";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }
    // std::cout << "estimate" << std::endl;
    // for (int i=0; i<3; i++) {
    //     for (int j=0; j<1; j++) {
    //         std::cout << (this->x).get(i, j) << "\t";
    //     }
    // }
    // std::cout << "\n";
    return this->x;
}