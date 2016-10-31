#include "navigation.h"
/**
  * @brief  Rotate the acc from b to i
  * @param  float *accbx,float *accby,float *accbz,float *accix,float *acciy,float *acciz,Angle*angle
  * @retval None
  */
void Refer_coordacc(Vector *acc_b,Vector *acc_i,Angle*angle)
{
	Vector tmpb,tmpi;
	Eular e;
	e.pitch=angle->pitch;
	e.roll=angle->roll ;
	e.yaw=angle->yaw;
	tmpb.x=acc_b->x;
	tmpb.y=acc_b->y;
	tmpb.z=acc_b->z;
	tmpi.x=acc_i->x;
	tmpi.y=acc_i->y;
	tmpi.z=acc_i->z;
	
	Eular_RoateVectXY(&tmpb,&tmpi,&e);
	acc_i->x=tmpi.x;
	acc_i->y=tmpi.y;
	acc_i->z=tmpi.z;
}
/**
  * @brief  smooth the throttle
  * @param  float *accbx,float *accby,float *accbz,float *accix,float *acciy,float *acciz,Angle*angle
  * @retval None
  */
void Thr_Smooth(Vector *acc_b,Vector *acc_i,float*pensate_Thr,Angle*angle,float dt)
{
	Refer_coordacc(acc_b,acc_i,angle);
	*pensate_Thr=DLPF(acc_i->z, *pensate_Thr, 10, 2*dt);
}
