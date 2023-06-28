#include <iostream>
#include "kalman_filter.h"
#include "math.h"

Kalman_filter::Kalman_filter()
{
    this->x = ModelMatrix(3,1);
    this->P = ModelMatrix::one(3,3);
    this->A = ModelMatrix::one(3,3);
    this->H = ModelMatrix::one(3,3);
    
    this->Q = ModelMatrix::one(3,3);
    this->Q = this->Q * 0.001;

    this->R = ModelMatrix::one(3,3);
    this->R = this->R * 0.001;
}


Kalman_filter::Kalman_filter(ModelMatrix x, ModelMatrix P)
{
    this->x = x;
    this->P = P;
    this->A = ModelMatrix::identity(3,3);
    this->B = ModelMatrix::identity(3,3);
    this->H = ModelMatrix::identity(3,3);
    
    this->Q = ModelMatrix::identity(3,3);
    this->Q = this->Q * 0.001;

    this->R = ModelMatrix::zero(3,3);
    // this->R = this->R * 0.01;
    std::cout << "kalman init" << std::endl;
}

ModelMatrix Kalman_filter::predict(ModelMatrix input, double dt)
{
    this->B.set(0,1, cos(this->x.get(2,0).to_double()) * dt);
    this->B.set(1,1, sin(this->x.get(2,0).to_double()) * dt);
    this->B.set(2,2, dt);
    // std::cout <<"kalman predict B = " << this->B.get(0,0).to_double() << " " << this->B.get(1,1).to_double() << " " << this->B.get(2,2).to_double() << std::endl;
    ModelMatrix past = this->x;
    this->x = this->A * past + this->B * input;
    this->P = this->A * this->P * this->A.transpose() + this->Q;
    
    return this->x;
}

ModelMatrix Kalman_filter::mesurement(ModelMatrix mesurement) {
    ModelMatrix K = this->P * this->H.transpose() * (this->H * this->P * this->H.transpose() + this->R).inverse();
    ModelMatrix fdf = K * (mesurement - H * this->x);
    this->x = this->x + fdf;
    this->P = this->P - K * this->H * this->P;

    return this->x;
}
