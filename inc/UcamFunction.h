#ifndef __UCAMFUNCTION_H
#define __UCAMFUNCTION_H



typedef enum{
    UCAM_SUCCESS = 0,
    UCAM_SYNC_FAIL = -1,
    UCAM_ACK_FAIL = -2
} ucam_status;


uint32_t delay_timer;
uint32_t tictoc_timer;

ucam_status ucam_init(void);
ucam_status ucam_get_picture(uint8_t * im);


ucam_status ucam_sync(void);
ucam_status ucam_im_setup(void);


void ucam_reset_pin_setup(void);
void ucam_reset(void);

void delay_ms(uint32_t time_ms);
void delay_100us(uint32_t time_100us);
void init_delay_timer(void);

void SysTick_Handler(void);








#endif /* __UCAMFUNCTION_H */
