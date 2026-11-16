#pragma once
static const void (*p_cpu_wakeup_init)(cap_typedef_e cap) = (void*)0x00016798;
#define cpu_wakeup_init  (*p_cpu_wakeup_init)
static const void (*p_clock_init)(SYS_CLK_TypeDef SYS_CLK) = (void*)0x00014f54;
#define clock_init  (*p_clock_init)
static const void (*p_user_read_efuse_value_calib)(void) = (void*)0x000103a0;
#define user_read_efuse_value_calib  (*p_user_read_efuse_value_calib)
static const void (*p_rf_mode_init)(void) = (void*)0x00015ea0;
#define rf_mode_init  (*p_rf_mode_init)
static const void (*p_rf_set_ble_1M_mode)(void) = (void*)0x00015f28;
#define rf_set_ble_1M_mode  (*p_rf_set_ble_1M_mode)
static const void (*p_rf_set_ble_1M_NO_PN_mode)() = (void*)0x0001606c;
#define rf_set_ble_1M_NO_PN_mode  (*p_rf_set_ble_1M_NO_PN_mode)
static const void (*p_rf_set_ble_2M_mode)() = (void*)0x0001610c;
#define rf_set_ble_2M_mode  (*p_rf_set_ble_2M_mode)
static const void (*p_rf_set_ble_2M_NO_PN_mode)() = (void*)0x00015fc8;
#define rf_set_ble_2M_NO_PN_mode  (*p_rf_set_ble_2M_NO_PN_mode)
static const void (*p_rf_set_pri_250K_mode)() = (void*)0x000161b4;
#define rf_set_pri_250K_mode  (*p_rf_set_pri_250K_mode)
static const void (*p_rf_set_pri_500K_mode)() = (void*)0x0001625c;
#define rf_set_pri_500K_mode  (*p_rf_set_pri_500K_mode)
static const void (*p_rf_set_pri_1M_mode)() = (void*)0x000162f8;
#define rf_set_pri_1M_mode  (*p_rf_set_pri_1M_mode)
static const void (*p_rf_set_pri_2M_mode)() = (void*)0x000163a0;
#define rf_set_pri_2M_mode  (*p_rf_set_pri_2M_mode)
static const void (*p_rf_set_channel_singletone) (signed char chn) = (void*)0x000158e4;
#define rf_set_channel_singletone  (*p_rf_set_channel_singletone)
static const void (*p_rf_set_power_level_index_singletone) (RF_PowerTypeDef level) = (void*)0x00015464;
#define rf_set_power_level_index_singletone  (*p_rf_set_power_level_index_singletone)
static const void (*p_rf_access_code_comm) (unsigned int acc) = (void*)0x0001599c;
#define rf_access_code_comm  (*p_rf_access_code_comm)
static const void (*p_rf_emi_single_tone)(RF_PowerTypeDef power_level,signed char rf_chn) = (void*)0x00015970;
#define rf_emi_single_tone  (*p_rf_emi_single_tone)
static const void (*p_rf_emi_stop)(void) = (void*)0x000154c0;
#define rf_emi_stop  (*p_rf_emi_stop)
static const void (*p_rf_emi_tx_continue_setup)(RF_ModeTypeDef rf_mode,RF_PowerTypeDef power_level,signed char rf_chn,unsigned char pkt_type) = (void*)0x000156d0;
#define rf_emi_tx_continue_setup  (*p_rf_emi_tx_continue_setup)
static const void (*p_rf_continue_mode_run)(void) = (void*)0x00015614;
#define rf_continue_mode_run  (*p_rf_continue_mode_run)
static const void (*p_rf_emi_rx)(RF_ModeTypeDef mode,signed char rf_chn) = (void*)0x000154dc;
#define rf_emi_rx  (*p_rf_emi_rx)
static const void (*p_rf_emi_rx_loop)(void) = (void*)0x000155a8;
#define rf_emi_rx_loop  (*p_rf_emi_rx_loop)
static const unsigned int (*p_rf_emi_get_rxpkt_cnt)(void) = (void*)0x00015608;
#define rf_emi_get_rxpkt_cnt  (*p_rf_emi_get_rxpkt_cnt)
static const void (*p_rf_emi_tx_burst_setup)(RF_ModeTypeDef rf_mode,RF_PowerTypeDef power_level,signed char rf_chn,unsigned char pkt_type) = (void*)0x00015770;
#define rf_emi_tx_burst_setup  (*p_rf_emi_tx_burst_setup)
static const void (*p_rf_emi_tx_burst_loop)(RF_ModeTypeDef rf_mode,unsigned char pkt_type) = (void*)0x0001583c;
#define rf_emi_tx_burst_loop  (*p_rf_emi_tx_burst_loop)
static const void (*p_gpio_set_func)(GPIO_PinTypeDef pin, gpio_func_e func) = (void*)0x00016e38;
#define gpio_set_func  (*p_gpio_set_func)
static const void (*p_gpio_set_input_en)(GPIO_PinTypeDef pin, unsigned int value) = (void*)0x00016ee4;
#define gpio_set_input_en  (*p_gpio_set_input_en)
static const void (*p_gpio_set_output_en)(GPIO_PinTypeDef pin, unsigned int value) = (void*)0x00016db0;
#define gpio_set_output_en  (*p_gpio_set_output_en)
static const void (*p_gpio_write)(GPIO_PinTypeDef pin, unsigned int value) = (void*)0x00016dec;
#define gpio_write  (*p_gpio_write)
static const _Bool (*p_gpio_read)(GPIO_PinTypeDef pin) = (void*)0x00016e18;
#define gpio_read  (*p_gpio_read)
static const void (*p_gpio_setup_up_down_resistor)(GPIO_PinTypeDef gpio, GPIO_PullTypeDef up_down) = (void*)0x00017034;
#define gpio_setup_up_down_resistor  (*p_gpio_setup_up_down_resistor)
