#include <model_matrix.h>

/*
current state 넣어주고, input 인자 넣어주고, 그러면 내부에서 알아서 예측값 주고, P 계산하고
*/

class Kalman_filter
{
public:
    Kalman_filter();
    Kalman_filter(ModelMatrix x, ModelMatrix P);
    ModelMatrix predict(ModelMatrix input, double dt);
    ModelMatrix mesurement(ModelMatrix mesurement);
    ModelMatrix x;

private:
    ModelMatrix P;
    ModelMatrix A;
    ModelMatrix B;
    ModelMatrix Q;
    ModelMatrix R;
    ModelMatrix H;
};