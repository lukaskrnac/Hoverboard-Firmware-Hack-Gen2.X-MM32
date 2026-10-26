/*-------------------- Includes -----------------------*/
#include "../Inc/mcu.h"
#include "../Inc/ipark.h"
#include "../Inc/FOC_Math.h"

/*------------------- Private variables ---------------*/
IPARK_T ipark1 = IPARK_DEFAULTS;

/*------------------ Private functions ----------------*/
void IPARK_MACRO1(IPARK_T *u1);


/*************************************
	Function Name：IPARK_MACRO1
	Description：This function transforms Vd、Vq to Valpha、Vbeta.
	Input：*u1--- Structure PARK_T 
	Output：none
**************************************/
void IPARK_MACRO1(IPARK_T *u1)											
{			
	Trig_Components Local_Vector_Components;
	
	Local_Vector_Components = MCM_Trig_Functions(u1->Theta);
	u1->Sin = Local_Vector_Components.hSin;
	u1->Cos = Local_Vector_Components.hCos;
	
	u1->Alpha = (u1->Ds * u1->Cos - u1->Qs * u1->Sin) >> 15;		//Valpha = -VQ*Sin + VD*Cos
	u1->Beta  = (u1->Qs * u1->Cos + u1->Ds * u1->Sin) >> 15;		//Vbeta = VQ*Cos + VD*Sin
}


