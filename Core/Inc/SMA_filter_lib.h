#include "main.h"

/* Макросы -------------------------------------------------------------------------------------------------------------------------------*/
#define SMA_FILTER_ORDER 32 /* Порядок SMA фильтра */

/* Выберите версию фильтра(Делают они одно и то же, просто разными способами. Какой из них оптимальнее - решать Вам)*/
#define SMA_VERSION_1
//#define SMA_VERSION_2


/* Прототипы функций ----------------------------------------------------------------------------------------------------------------------*/

#if defined (SMA_VERSION_1)
	uint16_t SMA_FILTER_Get_Value(uint16_t *SMA_Filter_buffer, uint16_t *RAW_Data);
#elif defined (SMA_VERSION_2)
	uint16_t SMA_FILTER_Get_Value(uint8_t *SMA_Filter_counter, uint16_t *SMA_Filter_buffer, uint32_t *SMA_Filter_Result, uint16_t *RAW_Data);
#endif



#ifdef __cplusplus
}
#endif

