#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#define M_PI    3.14159265358979323846
#define M_PI_2  1.57079632679489661923

typedef struct WayPoint_ {
    float x;
    float y;

    WayPoint_(float x, float y)
    {
        this->x = x;
        this->y = y;
    }

} WayPoint;

typedef struct Point_ {
    float x;
    float y;
    float yaw;
    float k;
    float speed;
} Point;


typedef struct ControlState_
{
    float x;
    float y;
    float yaw;
    float steer;
    float v;

    ControlState_()
    {

    }

    ControlState_(float x, float y, float yaw, float steer, float v)
    {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->steer = steer;
        this->v = v;
    }
} ControlState;

#endif
