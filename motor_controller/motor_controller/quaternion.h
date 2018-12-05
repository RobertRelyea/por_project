#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct Quaternion_t
{
  double w;
  double x;
  double y;
  double z;
}Quaternion;

Quaternion toQuaternion(double yaw, double pitch, double roll); // yaw (Z), pitch (Y), roll (X)



#endif
