#ifndef __UCAMFUNCTION_H
#define __UCAMFUNCTION_H



typedef enum{
    UCAM_SUCCESS = 0,
    UCAM_SYNC_FAIL = -1,
    UCAM_ACK_FAIL = -2
} ucam_status;


uint32_t delay_timer;

ucam_status ucam_init(void);
ucam_status ucam_sync(void);

void ucam_reset_pin_setup(void);
void ucam_reset(void);

void delay_ms(uint32_t time_ms);
void init_delay_ms(void);

void SysTick_Handler(void);








#endif /* __UCAMFUNCTION_H */
