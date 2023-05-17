#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H


#define M_PI    3.14159265358979323846
#define M_PI_2  1.57079632679489661923

typedef struct WayPoint_
{
    double x;
    double y;

    WayPoint_(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

} WayPoint;

typedef struct Point_
{
    double x;
    double y;
    double yaw;
    double k;
    double speed;
} Point;


typedef struct ControlState_
{
    double x;
    double y;
    double yaw;
    double steer;
    double v;

    ControlState_()
    {

    }

    ControlState_(double x, double y, double yaw, double steer, double v)
    {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->steer = steer;
        this->v = v;
    }
} ControlState;

#endif
