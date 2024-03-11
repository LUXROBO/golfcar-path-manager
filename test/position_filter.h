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
    position_filter(ModelMatrix x, ModelMatrix P);
    ModelMatrix state_equation_jacobi(ModelMatrix x0, ModelMatrix input);
    ModelMatrix predict_position(ModelMatrix x0, ModelMatrix input);
    ModelMatrix predict(ModelMatrix input);
    ModelMatrix estimate_yaw_with_imu(ModelMatrix z);
    ModelMatrix estimate_xy_with_gps(ModelMatrix z, int quality);
    ModelMatrix estimate_yaw_with_gps(ModelMatrix z, int quality);
    ModelMatrix estimate(ModelMatrix z);
    
    void set_x(ModelMatrix x)
    {
        this->x = x;
        this->init_flag = true;
    }

    ModelMatrix get_x()
    {
        return x;
    }

    void set_H(ModelMatrix H)
    {
        this->H = H;
    };

    void set_R(ModelMatrix R)
    {
        this->R = R;
    };

private:
    // ModelMatrix A;  /** 상태 전이 함수 */
    ModelMatrix H;          /** 측정 상태 공간 방정식 */
    ModelMatrix P;          /** 측정 에러 공분산*/
    ModelMatrix P_predict;  /** 측정 에러 예측 공분산*/
    ModelMatrix K;          /** 칼만 게인 */
    ModelMatrix x;          /** state */
    ModelMatrix x_predict;  /** 예측 state*/
    ModelMatrix Q;          /** 예측 노이즈*/
    ModelMatrix R;          /** 측정 노이즈*/

    bool init_flag;

    // int (*Ajacob)(int, int);
    // int (*Hjacob)(int, int);
    // int (*Ajacob)(int, int);
};


