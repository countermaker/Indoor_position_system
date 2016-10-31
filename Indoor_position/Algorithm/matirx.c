#include "mcu.h"
/*data type*/
typedef struct
{
	float a11;
	float a12;
	float a13;
	float a21;
	float a22;
	float a23;
	float a31;
	float a32;
	float a33;
}MAT_3X3;

typedef struct
{
	float a11;
	float a12;
	float a21;
	float a22;
}MAT_2X2;

#define A11 (A->a11)
#define A12 (A->a12)
#define A13 (A->a13)
#define A21 (A->a21)
#define A22 (A->a22)
#define A23 (A->a23)
#define A31 (A->a31)
#define A32 (A->a32)
#define A33 (A->a33)

#define SUCCESS 1
#define FAULS 0
typedef unsigned char flag;

MAT_3X3 F,F_Invert;
/**
  * @brief  C = A + B
  * @param  MAT_3X3*A,MAT_3X3*B,MAT_3X3*C
  * @retval None
  */
void Matrix3X3_Add(MAT_3X3*A,MAT_3X3*B,MAT_3X3*C)
{
	C->a11 = A->a11 + B->a11;
	C->a12 = A->a12 + B->a12;
	C->a13 = A->a13 + B->a13;
	C->a21 = A->a21 + B->a21;
	C->a22 = A->a22 + B->a22;
	C->a23 = A->a23 + B->a23;
	C->a31 = A->a31 + B->a31;
	C->a32 = A->a32 + B->a32;
	C->a33 = A->a33 + B->a33;
}

/**
  * @brief  C = A - B
  * @param  MAT_3X3*A,MAT_3X3*B,MAT_3X3*C
  * @retval None
  */
void Matrix3X3_Sub(MAT_3X3*A,MAT_3X3*B,MAT_3X3*C)
{
	C->a11 = A->a11 - B->a11;
	C->a12 = A->a12 - B->a12;
	C->a13 = A->a13 - B->a13;
	C->a21 = A->a21 - B->a21;
	C->a22 = A->a22 - B->a22;
	C->a23 = A->a23 - B->a23;
	C->a31 = A->a31 - B->a31;
	C->a32 = A->a32 - B->a32;
	C->a33 = A->a33 - B->a33;
}
/**
  * @brief  C = A * B
  * @param  MAT_3X3*A,MAT_3X3*B,MAT_3X3*C
  * @retval None
  */
void Matrix3X3_Dot(MAT_3X3*A,MAT_3X3*B,MAT_3X3*C)
{
	C->a11 = A->a11 * B->a11 + A->a12 * B->a21 + A->a13 * B->a31;
	C->a12 = A->a11 * B->a12 + A->a12 * B->a22 + A->a13 * B->a32;
	C->a13 = A->a11 * B->a13 + A->a12 * B->a23 + A->a13 * B->a33;
	C->a21 = A->a21 * B->a11 + A->a22 * B->a21 + A->a23 * B->a31;
	C->a22 = A->a21 * B->a12 + A->a22 * B->a22 + A->a23 * B->a32;
	C->a23 = A->a21 * B->a13 + A->a22 * B->a23 + A->a23 * B->a33;
	C->a31 = A->a31 * B->a11 + A->a32 * B->a21 + A->a33 * B->a31;
	C->a32 = A->a31 * B->a12 + A->a32 * B->a22 + A->a33 * B->a32;
	C->a33 = A->a31 * B->a13 + A->a32 * B->a23 + A->a33 * B->a33;
}
/**
  * @brief  C = A /c
  * @param  MAT_3X3*A
  * @retval None
  */
void Matrix3X3_Eli(MAT_3X3*A,MAT_3X3*C,float c)
{
	C->a11 = A->a11 / c;
	C->a12 = A->a12 / c;
	C->a13 = A->a13 / c;
	C->a21 = A->a21 / c;
	C->a22 = A->a22 / c; 
	C->a23 = A->a23 / c;
	C->a31 = A->a31 / c;
	C->a32 = A->a32 / c;
	C->a33 = A->a33 / c;
}
/**
  * @brief  Get the determinant of matirx A
  * @param  AT_3X3*A
  * @retval det
  */
float Matrix3X3_Getdet(MAT_3X3*A)
{
	float det;
	float det_tmp1,det_tmp2;
	det_tmp1 = A11 * A22 * A33 + A12 * A23 * A31 + A13 * A21 * A32;
	det_tmp2 = A31 * A22 * A13 - A32 * A23 * A11 + A33 * A21 * A12;
	det = det_tmp1 - det_tmp2;
	return det;
}
/**
  * @brief  Get the transpose of matirx A B = A_T
  * @param  AT_3X3*A AT_3X3*A_T
  * @retval none
  */
void Matrix3X3_Gettranspose(MAT_3X3*A,MAT_3X3*A_T)
{
	A_T->a11 = A->a11;
	A_T->a12 = A->a21;
	A_T->a13 = A->a31;
	A_T->a21 = A->a12;
	A_T->a22 = A->a22;
	A_T->a23 = A->a32;
	A_T->a31 = A->a13;
	A_T->a32 = A->a23;
	A_T->a33 = A->a33;
}
/**
  * @brief  Get the determinant of matirx A
  * @param  AT_3X3*A
  * @retval det
  */
float Matrix2X2_Getdet(MAT_2X2*A)
{
	float det;
	det = A11 * A22 - A12 * A21;
	return det;
}
/**
  * @brief  Get the cofactor of A
  * @param  AT_3X3*A
  * @retval det
  */
void Matrix2X2_Getcof(MAT_3X3*A,MAT_2X2*M11,MAT_2X2*M12,MAT_2X2*M13,MAT_2X2*M21,
	MAT_2X2*M22,MAT_2X2*M23,MAT_2X2*M31,MAT_2X2*M32,MAT_2X2*M33)
{
	M11->a11 = A->a22;
	M11->a12 = A->a23;
	M11->a21 = A->a32;
	M11->a22 = A->a33;
	
	M12->a11 = A->a21;
	M12->a12 = A->a23;
	M12->a21 = A->a31;
	M12->a22 = A->a33;
	
	M13->a11 = A->a21;
	M13->a12 = A->a22;
	M13->a21 = A->a31;
	M13->a22 = A->a32;
	
	M21->a11 = A->a12;
	M21->a12 = A->a13;
	M21->a21 = A->a32;
	M21->a22 = A->a33;
	
	M22->a11 = A->a11;
	M22->a12 = A->a13;
	M22->a21 = A->a31;
	M22->a22 = A->a33;
	
	M23->a11 = A->a11;
	M23->a12 = A->a12;
	M23->a21 = A->a31;
	M23->a22 = A->a32;
	
	M31->a11 = A->a12;
	M31->a12 = A->a13;
	M31->a21 = A->a22;
	M31->a22 = A->a23;
	
	M32->a11 = A->a11;
	M32->a12 = A->a13;
	M32->a21 = A->a21;
	M32->a22 = A->a23;
	
	M33->a11 = A->a11;
	M33->a12 = A->a12;
	M33->a21 = A->a21;
	M33->a22 = A->a22;
}
/**
  * @brief  Get the Inverse matirx of A
  * @param  AT_3X3*A AT_3X3*B
  * @retval flag SUCCESS FAULS
  */
flag Matrix3X3_GetInverse(MAT_3X3*A,MAT_3X3*B)
{
	MAT_2X2 M11,M12,M13,M21,M22,M23,M31,M32,M33;
	MAT_3X3 A_T;
	MAT_3X3 M;
	float det;
	float det_11,det_12,det_13,det_21,det_22,det_23,det_31,det_32,det_33;
	Matrix3X3_Gettranspose(A,&A_T);
	det = Matrix3X3_Getdet(&A_T);

	if(det == 0)
	{
		return FAULS;
	}
	Matrix2X2_Getcof(&A_T,&M11,&M12,&M13,&M21,&M22,&M23,&M31,&M32,&M33);

	det_11 = Matrix2X2_Getdet(&M11);
	det_12 = Matrix2X2_Getdet(&M12);
	det_13 = Matrix2X2_Getdet(&M13);
	det_21 = Matrix2X2_Getdet(&M21);
	det_22 = Matrix2X2_Getdet(&M22);
	det_23 = Matrix2X2_Getdet(&M23);
	det_31 = Matrix2X2_Getdet(&M31);
	det_32 = Matrix2X2_Getdet(&M32);
	det_33 = Matrix2X2_Getdet(&M33);

	M.a11 = det_11;
	M.a12 = det_12;
	M.a13 = det_13;
	M.a21 = det_21;
	M.a22 = det_22;
	M.a23 = det_23;
	M.a31 = det_31;
	M.a32 = det_32;
	M.a33 = det_33;	
	Matrix3X3_Eli(&M,B,det);
 
 	return SUCCESS;
}

















