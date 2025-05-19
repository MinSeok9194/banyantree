/*******************************(C) COPYRIGHT 2007 INSEM Inc.****************************************/
/* processor 	  : CORETEX-M3(STM32F10X)         		    				   */
/* compiler     : EWARM Compiler								    				*/
/* program by	  : H.H.HWANG									    						*/
/* History		  : 								    									*/
/* copy right	  : PLUS-H									    					*/
/****************************************************************************************************/

// Edit date = 2014. 11. 25
// 1. 부전동 웨스트 호텔 샘플룸 적용
// 2. 통신형 K/S의 스위치 기능 변경 = 전열교환기, 입구등으로...
//

#define __MAIN_H__  

#include "stm32f10x_lib.h"   
#include "main.h"

/*******************************************************************************
* Function Name  : maing
* Description    : Main program.
* Input          : None
* Output         : Nonet
* Return         : None
*******************************************************************************/
u8 proctrl =7;int main(void) {
#ifdef DEBUG
  debug();
#endif
        
	CORTEX_initial();      
	while(1){
                
 		tcrst =~tcrst;   
                
                UART1_control_proc();
                
		switch(proctrl){
			case 0:				
        UART2_control_proc();
      	proctrl++;       
        break; 
        
			case 1:   
        Internal_timer_Proc(); 				
        internal_exec_proc();
				proctrl++;       
        break; 
        
			case 2:
        UART3_control_proc();
				proctrl=0;		    
        break; 
        
        default:		proctrl =0;		break;              
		}  
        
	}
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1) { 
  }
}
#endif

/******************* (C) COPYRIGHT 2007 INSEM Inc ***************************************END OF FILE****/

