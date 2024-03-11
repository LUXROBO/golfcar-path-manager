#pragma once

#include <path_manager.h>
#include <model_matrix_double.h>
#include <qformat.h>

// std
#include <stddef.h>
#include <stdint.h>


class position_filter
{
public:
    position_filter();
    ~position_filter();
    position_filter(ModelMatrix_D x, ModelMatrix_D P);
    ModelMatrix_D state_equation_jacobi(ModelMatrix_D x0, ModelMatrix_D input);
    ModelMatrix_D predict_position(ModelMatrix_D x0, ModelMatrix_D input);
    ModelMatrix_D predict(ModelMatrix_D input);
    ModelMatrix_D estimate_yaw_with_imu(ModelMatrix_D z);
    ModelMatrix_D estimate_xy_with_gps(ModelMatrix_D z, int quality);
    ModelMatrix_D estimate_yaw_with_gps(ModelMatrix_D z, int quality);
    ModelMatrix_D estimate(ModelMatrix_D z);

    void set_x(ModelMatrix_D x)
    {
        this->x = x;
        this->init_flag = true;
    };

    ModelMatrix_D get_x()
    {
        return x;
    };

    void set_H(ModelMatrix_D H)
    {
        this->H = H;
    };

    void set_R(ModelMatrix_D R)
    {
        this->R = R;
    };

private:
    // ModelMatrix_D A;  /** 상태 전이 함수 */
    ModelMatrix_D H;          /** 측정 상태 공간 방정식 */
    ModelMatrix_D P;          /** 측정 에러 공분산 */
    ModelMatrix_D P_predict;  /** 측정 에러 예측 공분산 */
    ModelMatrix_D K;          /** 칼만 게인 */
    ModelMatrix_D x;          /** state */
    ModelMatrix_D x_predict;  /** 예측 state */
    ModelMatrix_D Q;          /** 예측 노이즈 */
    ModelMatrix_D R;          /** 측정 노이즈 */

    bool init_flag;
};
