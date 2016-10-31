#include "rotate.h"

/**
  * @brief  Copy the vector
  * @param  Vector *a, Vector *b
	          b=a;
  * @retval None
  */
void Vect_Copy(Vector *a, Vector *b)
{
  b->x = a->x;
  b->y = a->y;
  b->z = a->z;
}
/**
  * @brief  Vector addition
  * @param  Vector *a, Vector *b
	          result = *a + *b
  * @retval None
  */
void Vect_Add(Vector *result , Vector *a, Vector *b)
{
  Vector tmp_a, tmp_b;
  Vect_Copy(a, &tmp_a);
  Vect_Copy(b, &tmp_b);
  result->x = tmp_a.x + tmp_b.x;
  result->y = tmp_a.y + tmp_b.y;
  result->z = tmp_a.z + tmp_b.z;
}
/**
  * @brief  Vector substraction
  * @param  Vector *a, Vector *b
	          result = *a - *b
  * @retval None
  */
void Vect_Sub(Vector *retult,Vector *a,Vector *b)
{
	Vector tmp_a,tmp_b;
	Vect_Copy(a,&tmp_a);
	Vect_Copy(b,&tmp_b);
	retult->x = tmp_a.x - tmp_b.x;
	retult->y = tmp_a.y - tmp_b.y;
	retult->z = tmp_a.z - tmp_b.z;
}
/**
  * @brief  Vector Scale
  * @param  float a, Vector *v
	          v *= a
  * @retval None
  */
void Vect_Scale(Vector *v , const float a)
{
  v->x = (v->x) * a;
  v->y = (v->y) * a;
  v->z = (v->z) * a;
}
/**
  * @brief  Vector Normalization
  * @param  Vector *v
  * @retval None
  */
void Vect_Normalize(Vector *v)
{
	float norm_inv = 1.0f/sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
	(v->x) *= norm_inv;
	(v->y) *= norm_inv;
	(v->z) *= norm_inv;
}
/**
  * @brief  |v|
  * @param  Vector *v
  * @retval None
  */
float Vect_Norm(Vector *v)
{
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}
/**
  * @brief  Vector cross product
  * @param  Vector *result Vector a, Vector b
						*result = a * b
  * @retval None
  */
void Vect_CrossProduct(Vector *result,Vector a,Vector b)
{
	result->x = a.y * b.z - a.z * b.y;
  result->y = a.z * b.x - a.x * b.z;
  result->z = a.x * b.y - a.y * b.x;
}
/**
  * @brief  rotate a small angle   R(Referent Coordinate)->b(quadcopter coordinate)
  * @param  Vector *a, Vector *b
	          b=a;
  * @retval None
  */
void Eular_RoateVectSmall(Vector *vect,Eular *ypr)
{
	Vector tmp;
	Vect_Copy(vect,&tmp);
	
	vect->x =               tmp.x + ypr->yaw  * tmp.y - ypr->pitch * tmp.z;
  vect->y =   -ypr->yaw * tmp.x +             tmp.y + ypr->roll  * tmp.z;
  vect->z =  ypr->pitch * tmp.x - ypr->roll * tmp.y +              tmp.z;
}
/**
  * @brief  rotate an angle		b(quadcopter coordinate)->R(Referent Coordinate)
  * @param  Vector *from, Vector *to,Eular*e
  * @retval None
  */
void Eular_RoateVect(Vector *from,Vector *to,Eular *e)
{
	float sin_yaw,cos_yaw;
	float sin_pitch,cos_pitch;
	float sin_roll,cos_roll;
	Vector tmp;
	
	Vect_Copy(from, &tmp);
	fast_SinCos(e->yaw, &sin_yaw, &cos_yaw);
	fast_SinCos(e->pitch, &sin_pitch, &cos_pitch);
  fast_SinCos(e->roll, &sin_roll, &cos_roll);
	
  to->x = tmp.x * (cos_yaw * cos_pitch) + tmp.y * (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) + tmp.z * (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll);
  to->y = tmp.x * (sin_yaw * cos_pitch) + tmp.y * (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) + tmp.z * (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll);
  to->z = tmp.x * (-sin_pitch)       		+ tmp.y * (cos_pitch * sin_roll)                         				+ tmp.z * (cos_pitch * cos_roll);
	
}
/**
  * @brief  rotate an angle XY b(quadcopter coordinate)->R(Referent Coordinate)
  * @param  Vector *from, Vector *to,Eular*e
  * @retval None
  */
void Eular_RoateVectXY(Vector *from,Vector *to,Eular *e)
{
	float sin_pitch,cos_pitch;
	float sin_roll,cos_roll;
	Vector tmp;
	Vect_Copy(from, &tmp);
	
	fast_SinCos(e->pitch, &sin_pitch, &cos_pitch);
  fast_SinCos(e->roll, &sin_roll, &cos_roll);
	
	to->x = tmp.x * cos_pitch + tmp.y * (sin_pitch * sin_roll) + tmp.z * (sin_pitch * cos_roll);
  to->y = tmp.y * (cos_roll) + tmp.z * (-sin_roll);
  to->z = tmp.x * (-sin_pitch)+ tmp.y * (cos_pitch * sin_roll) + tmp.z * (cos_pitch * cos_roll);
}

void Quaternion_Normalize(Quaternion *q)
{
	float norm_inv = 1.0f / sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
	(q->w) *= norm_inv;
	(q->x) *= norm_inv;
	(q->y) *= norm_inv;
	(q->z) *= norm_inv;
}
void Quaternion_Conj(Quaternion *result,Quaternion *q)
{
	result->w = q->w;
	result->x = -q->x;
	result->y = -q->y;
	result->z = -q->z;
}

void Quaternion_Copy(Quaternion *result,Quaternion *q)
{
	result->w = q->w;
	result->x = q->x;
	result->y = q->y;
	result->z = q->z;
}

void Quaternion_Mul(Quaternion *result,const Quaternion q_left,const Quaternion q_right)
{
	result->w = q_left.w * q_right.w - q_left.x * q_right.x - q_left.y * q_right.y - q_left.z * q_right.z;
  result->x = q_left.x * q_right.w + q_left.w * q_right.x + q_left.y * q_right.z - q_left.z * q_right.y;
  result->y = q_left.y * q_right.w + q_left.w * q_right.y + q_left.z * q_right.x - q_left.x * q_right.z;
  result->z = q_left.z * q_right.w + q_left.w * q_right.z + q_left.x * q_right.y - q_left.y * q_right.x;
}
void Quaternion_Get(Quaternion *q,Vector from,Vector to)
{
	float from_inv_norm = 1.0f / sqrt(from.x * from.x + from.y * from.y + from.z * from.z);
  float to_inv_norm   = 1.0f / sqrt(to.x * to.x + to.y * to.y + to.z * to.z);
	
	Vector w;
  float norm_w_inv;
  float cos_theta;
  float sin_half_theta,cos_half_theta;
	
	cos_theta = (from.x * to.x + from.y * to.y + from.z * to.z) * (from_inv_norm * to_inv_norm);

	cos_half_theta = sqrtf((1.0f + cos_theta) / 2.0f);
  sin_half_theta = sqrt((1.0f - cos_theta) * 0.5f);
  /* w = from x to */
  Vect_CrossProduct(&w, from, to);

  norm_w_inv = 1.0f / sqrt(w.x * w.x + w.y * w.y + w.z * w.z);

  q->w = cos_half_theta;
  q->x = w.x * norm_w_inv * sin_half_theta;
  q->y = w.y * norm_w_inv * sin_half_theta;
  q->z = w.z * norm_w_inv * sin_half_theta;
}
/*b(quadcopter coordinate)->R(Referent Coordinate) a->b*/
void Quaternion_RotateVet1(const Quaternion *rotation , Vector *a, Vector *b)
{
  Vector from;
  float _2x  = (rotation->x) * 2.0f;
  float _2y  = (rotation->y) * 2.0f;
  float _2z  = (rotation->z) * 2.0f;
  float w_2x = (rotation->w) *  _2x;
  float w_2y = (rotation->w) *  _2y;
  float w_2z = (rotation->w) *  _2z;
  float x_2x = (rotation->x) *  _2x;
  float x_2y = (rotation->x) *  _2y;
  float x_2z = (rotation->x) *  _2z;
  float y_2y = (rotation->y) *  _2y;
  float y_2z = (rotation->y) *  _2z;
  float z_2z = (rotation->z) *  _2z;

  Vect_Copy(a, &from);

  b->x = (from.x) * (1.0f - y_2y - z_2z) + (from.y) * (x_2y - w_2z)        + (from.z) * (x_2z + w_2y);
  b->y = (from.x) * (x_2y + w_2z)        + (from.y) * (1.0f - x_2x - z_2z) + (from.z) * (y_2z - w_2x);
  b->z = (from.x) * (x_2z - w_2y)        + (from.y) * (y_2z + w_2x)        + (from.z) * (1.0f - x_2x - y_2y);
}

void Quaternion_RotateVet2(const Quaternion *rotation , const Vector *a, Vector *b)
{
  float _2x  = (rotation->x) * 2.0f;
  float _2y  = (rotation->y) * 2.0f;
  float _2z  = (rotation->z) * 2.0f;
  float w_2x = (rotation->w) *  _2x;
  float w_2y = (rotation->w) *  _2y;
  float w_2z = (rotation->w) *  _2z;
  float x_2x = (rotation->x) *  _2x;
  float x_2y = (rotation->x) *  _2y;
  float x_2z = (rotation->x) *  _2z;
  float y_2y = (rotation->y) *  _2y;
  float y_2z = (rotation->y) *  _2z;
  float z_2z = (rotation->z) *  _2z;


  b->x = (a->x) * (1.0f - y_2y - z_2z) + (a->y) * (x_2y + w_2z)        + (a->z) * (x_2z - w_2y);
  b->y = (a->x) * (x_2y - w_2z)        + (a->y) * (1.0f - x_2x - z_2z) + (a->z) * (y_2z + w_2x);
  b->z = (a->x) * (x_2z + w_2y)        + (a->y) * (y_2z - w_2x)        + (a->z) * (1.0f - x_2x - y_2y);
}

void Quaternion_ToVector(Quaternion *q,Vector *v)
{
	v->x = q->x;
  v->y = q->y;
  v->z = q->z;
}

void Quaternion_ConvertToEuler(Quaternion *q , Eular *e)
{
  e->yaw   = atan2(2.0f * (q->w) * (q->z) + 2.0f * (q->x) * (q->y), 1.0f - 2.0f * (q->y) * (q->y) - 2.0f * (q->z) * (q->z));
  e->pitch = asin(2.0f * (q->w) * (q->y) - 2.0f * (q->z) * (q->x));
  e->roll  = atan2(2.0f * (q->w) * (q->x) + (q->y) * (q->z) , 1.0f - 2.0f * (q->x) * (q->x) - 2.0f * (q->y) * (q->y));
}

void Eular_ConvertToQuaternion(Quaternion *q, Eular *e)
{
  float yaw_half, pitch_half, roll_half;
  float cos_yaw_half, sin_yaw_half;
  float cos_pitch_half, sin_pitch_half;
  float cos_roll_half, sin_roll_half;
  //  float q_norm_inv;
  //  QUATERNION q_tmp;
  yaw_half   = e->yaw * 0.5f;
  pitch_half = e->pitch * 0.5f;
  roll_half  = e->roll * 0.5f;

  fast_SinCos(yaw_half, &sin_yaw_half, &cos_yaw_half);
  fast_SinCos(pitch_half, &sin_pitch_half, &cos_pitch_half);
  fast_SinCos(roll_half, &sin_roll_half, &cos_roll_half);

  q->w = cos_roll_half * cos_pitch_half * cos_yaw_half + sin_roll_half * sin_pitch_half * sin_yaw_half;
  q->x = sin_roll_half * cos_pitch_half * cos_yaw_half - cos_roll_half * sin_pitch_half * sin_yaw_half;
  q->y = cos_roll_half * sin_pitch_half * cos_yaw_half + sin_roll_half * cos_pitch_half * sin_yaw_half;
  q->z = cos_roll_half * cos_pitch_half * sin_yaw_half - sin_roll_half * sin_pitch_half * cos_yaw_half;

  /* normalize */
  //  q_norm_inv = InvSqrt(q_tmp.w*q_tmp.w + q_tmp.x*q_tmp.x + q_tmp.y*q_tmp.y + q_tmp.z*q_tmp.z);
  //  q->w = q_tmp.w * q_norm_inv;
  //  q->x = q_tmp.x * q_norm_inv;
  //  q->y = q_tmp.y * q_norm_inv;
  //  q->z = q_tmp.z * q_norm_inv;
}
