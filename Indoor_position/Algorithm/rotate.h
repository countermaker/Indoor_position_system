#ifndef ROTATE_H_
#define ROTATE_H_
#include "mcu.h"
#include "cordic.h"
#include "global.h"

void Vect_Sub(Vector *retult,Vector *a,Vector *b);

void Vect_Copy(Vector *a, Vector *b);

void Vect_Scale(Vector *v , const float a);

void Vect_Add(Vector *result , Vector *a, Vector *b);

void Vect_Normalize(Vector *v);

void Eular_RoateVect(Vector*from,Vector*to,Eular*e);

void Eular_RoateVectSmall(Vector*vect,Eular*ypr);

void Eular_RoateVectXY(Vector*from,Vector*to,Eular*e);

void Quaternion_Normalize(Quaternion *q);

void Quaternion_Conj(Quaternion *result,Quaternion *q);

void Quaternion_Copy(Quaternion *result,Quaternion *q);

void Quaternion_Mul(Quaternion *result,const Quaternion q_left,const Quaternion q_right);

void Quaternion_Get(Quaternion *q,Vector from,Vector to);

void Quaternion_RotateVet1(const Quaternion *rotation , Vector *a, Vector *b);

void Quaternion_RotateVet2(const Quaternion *rotation , const Vector *a, Vector *b);

void Quaternion_ToVector(Quaternion *q,Vector *v);

void Quaternion_ConvertToEuler(Quaternion *q , Eular *e);

void Eular_ConvertToQuaternion(Quaternion *q, Eular *e);
#endif
