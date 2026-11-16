/********************************************************************************************************
 * @file     ota_boot.c
 *
 * @brief    Bootloader 内的 OTA 接收实现
 *
 *           将 OTA 完整流程（RF 收发 + OTA 状态机 + flash 写 + 固件搬运）移到 bootloader。
 *           bootloader 上电后判断 OTA 标记 / app 固件有效性，决定进入 OTA 模式或跳转 app。
 *
 * @note     精简自 app 侧 app_24g.c / app_24g_rx.c / app_24g_rf.c，只保留 OTA 必需代码。
 *******************************************************************************************************/

#include "app_public_config.h"
#include "toy_log.h"
#include "app_24g_rx.h"
#include "watchdog.h"
#include "ota_config.h"
#include "flash_fw_check.h"
#include "bootloader_ota/boot_copy_data.h"
#include "hal_driver/flash/hal_flash.h"
#include "sdk_version.h"

// 前向声明（定义在本文件末尾）
u8 boot_app_is_valid(void);
static u8 boot_ota_ready(void);

// ============================================================================
// OTA 标记区域：OTA_FLAG_ADDR / OTA_FLAG_MAGIC 由 ota_config.h 提供
// ============================================================================

// ============================================================================
// 全局变量（retention）
// ============================================================================
_attribute_data_retention_ User_Relevant_Var_T user_var;
_attribute_data_retention_ Pair_Relevant_Var_T pair_var;
_attribute_data_retention_ efuse_dev_info_t efuse_dev_info;

_attribute_data_retention_ u8  rf_state = 0;
_attribute_data_retention_ u8  device_channel = 0;
_attribute_data_retention_ u8  device_ack_received = 0;
_attribute_data_retention_ u32 last_rx_tick = 0;
_attribute_data_retention_ u8  rf_chn[5] = {0};

// OTA 状态变量
_attribute_data_retention_ u8  d24g_ota_status = 0;
_attribute_data_retention_ ota_buff_t ota_buff;
_attribute_data_retention_ u8  ota_buff_valid_flag = 0;
_attribute_data_retention_ u32 ota_program_offset = 0;
_attribute_data_retention_ u32 blt_ota_start_tick = 0;
_attribute_data_retention_ u8  d24g_ota_start_flag = 0;
_attribute_data_retention_ u8  ota_pno_ack_flag = 0;
_attribute_data_retention_ u16 ota_start_index = 0;
_attribute_data_retention_ u16 ota_expect_idx = 0;  // 期望的下一帧 DATA 序号（poll 上报 dongle，用于选择性重传）
_attribute_data_retention_ u32 ota_flash_write_addr = 0;
_attribute_data_retention_ u8  ota_first_data_buf[16];
_attribute_data_retention_ u32 ota_fw_size = 0;
_attribute_data_retention_ u8  ota_error_flag = 0;
_attribute_data_retention_ u8  ota_pending = 0;
_attribute_data_retention_ u32 ota_pending_version = 0;
_attribute_data_retention_ u8  ota_auto_tx_pending = 0;
_attribute_data_retention_ u8  ota_poll_active = 0;
_attribute_data_retention_ u8  ota_end_done = 0;      // 1 = END 校验通过，等 poll 回 pno=1 后搬运重启
_attribute_data_retention_ u8  ota_end_ack_sent = 0;  // 1 = END 确认（pno=1）的 poll 已发出，等 TX 完成即重启

// ============================================================================
// CRC16（Modbus 风格，与 dongle / 原 app 一致）
// ============================================================================
static const unsigned short crc16_poly1[2] = {0x0000, 0xa001};

unsigned short crc16_user(unsigned char *pD, int len)
{
    unsigned short crc = 0xffff;
    int i, j;
    for (j = len; j > 0; j--) {
        unsigned char ds = *pD++;
        for (i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ crc16_poly1[(crc ^ ds) & 1];
            ds = ds >> 1;
        }
    }
    return crc;
}

// ============================================================================
// OTA 标记读写
// ============================================================================
static u8 ota_flag_is_set(void)
{
    u32 flag = 0;
    flash_read_page(OTA_FLAG_ADDR, 4, (u8 *)&flag);
    return (flag == OTA_FLAG_MAGIC);
}

// 供 main 判断是否需要进入 OTA 模式
u8 boot_need_ota(void)
{
    u8 flag_set = ota_flag_is_set();
    u8 app_ok   = boot_app_is_valid();
    u8 ota_rdy  = boot_ota_ready();
    TOY_LOG("---boot_need_ota: flag=%d app_ok=%d ota_ready=%d -> %d\n",
           flag_set, app_ok, ota_rdy, flag_set || !app_ok || ota_rdy);
    return flag_set || !app_ok || ota_rdy;
}

static void ota_flag_clear(void)
{
    flash_erase_sector(OTA_FLAG_ADDR);
}

void ota_flag_set(void)  // 供 app 调用：写标记后重启
{
    hal_flash_unlock();
    flash_erase_sector(OTA_FLAG_ADDR);
    u32 magic = OTA_FLAG_MAGIC;
    flash_write_page(OTA_FLAG_ADDR, 4, (u8 *)&magic);
}

// ============================================================================
// RF 底层封装（精简自 app_24g_rf.c）
// ============================================================================
static inline u32 rf_access_code_24to32(u32 code)
{
    u32 r = (code & 0x00ffff00) >> 8;
    u32 p = code & 0xff;
    u32 t = 0;
    for (int i = 0; i < 8; i++) {
        t = t << 2;
        t |= ((p & BIT(i)) ? 1 : 2);
        t = t << 2;
        t |= (r & 0x03);
        r = r >> 2;
    }
    return t;
}

void rf_set_pair_access_code(u32 code)
{
    write_reg32(0x800408, ((code & 0xffffff00) | 0x71));
    write_reg8(0x80040c, code);
}

void rf_set_data_access_code(u32 code)
{
    write_reg32(0x800410, ((code & 0xffffff00) | 0x77));
    write_reg8(0x800414, code);
}

void rf_irq_src_allclr(void)
{
    reg_rf_irq_status = 0xffff;
}

void rf_start_stxtorx(void *addr, unsigned int tick, unsigned int timeout_us)
{
    write_reg16(0x800f0a, timeout_us - 1);
    write_reg32(0x800f18, tick);
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);
    write_reg16(0x800c0c, (unsigned short)((unsigned int)addr));
    write_reg8(0x800f00, 0x87);   // single tx2rx
}

// RF channel 固定：未配对时全部用 2402 MHz（dongle 广播/推送也在此信道）
#define FRE_OFFSET        0
#define DEFAULT_RF_CHAN   2   // 2402 MHz

static void set_unpaired_fixed_channel(void)
{
    unsigned char ch = (unsigned char)(FRE_OFFSET + DEFAULT_RF_CHAN);
    for (u8 i = 0; i < 5; i++) rf_chn[i] = ch;
}

void rf_drv_private_1m_init(void)
{
    rf_mode_init();
    rf_set_pri_1M_mode();

    write_reg16(0xf06, 0);          // rx wait time
    write_reg8(0x0f0c, 0x5c);       // rx settle time
    write_reg16(0xf0e, 0);          // tx wait time
    write_reg8(0x0f04, 0x70);       // tx settle time
    write_reg8(0xf10, 0);           // wait time on NAK

    user_var.rf_rx_wptr = 0;
    rf_rx_buffer_set((u8 *)(user_var.rf_rx_buff), PKT_BUFF_SIZE, 0);

    reg_dma_chn_en |= FLD_DMA_CHN_RF_RX | FLD_DMA_CHN_RF_TX;

    write_reg8(0x405, (read_reg8(0x405) & 0xF8) | 5);
    write_reg8(0x420, 38);
    write_reg8(0x402, 0x44);        // preamble 4 bytes

    rf_set_tx_rx_off();

    rf_rx_acc_code_enable(0x01);
    rf_tx_acc_code_select(0);

    irq_disable();
    irq_clr_src();
    rf_irq_src_allclr();
    irq_enable_type(FLD_IRQ_ZB_RT_EN);
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_TX | FLD_RF_IRQ_RX | FLD_RF_IRQ_RX_TIMEOUT | FLD_RF_IRQ_FIRST_TIMEOUT);
    irq_enable();
}

_attribute_ram_code_sec_ void rf_stx_to_rx(u8 *p, u32 rx_timeout_us)
{
    rf_state = RF_TX_START_STATUS;
    rf_set_tx_rx_off();
    device_ack_received = 0;
    rf_trx_state_set(RF_MODE_AUTO, rf_chn[device_channel]);
    rf_start_stxtorx(p, ClockTime(), rx_timeout_us);
    reg_rf_irq_status = 0xffff;
}

// ============================================================================
// RF 初始化 + 进入 RX（OTA 专用：固定 pair access code + 固定信道）
// ============================================================================
void custom_init(void)
{
    memset(&user_var, 0, sizeof(user_var));
    memset(&pair_var, 0, sizeof(pair_var));

    if (!efuse_read(EFUSE_CHIP_ID_ADDR, (u8 *)&efuse_dev_info, sizeof(efuse_dev_info_t))) {
        start_reboot();
    }
    u32 device_id = ((efuse_dev_info.chip_id[2] << 16) |
                     (efuse_dev_info.chip_id[1] << 8) |
                      efuse_dev_info.chip_id[0]);

    pair_var.pair_accesscode = pair_access_code_array[CAR_ACCESS_CODE_GROUP];
    user_var.dongle_id = rf_access_code_24to32(device_id);

    set_unpaired_fixed_channel();
    pair_var.pair_success_flag = 0;

    user_var.tx_power_normal = DEFAULT_NORMAL_TX_POWER;
}

void d24_user_init(void)
{
    rf_set_pair_access_code(pair_var.pair_accesscode);
    rf_set_data_access_code(user_var.dongle_id);
    rf_rx_acc_code_enable(0x03);
    rf_tx_acc_code_select(0);
    rf_set_power_level_index(user_var.tx_power_normal);
    rf_set_channel(rf_chn[user_var.host_channel], 0);
    rf_trx_state_set(RF_MODE_RX, rf_chn[user_var.host_channel]);
}

// ============================================================================
// OTA 命令响应（精简自 rf_ota_data_response）
// ============================================================================
u8 rf_ota_data_response(rf_packet_t *p_rf_rx_data)
{
    ota_data_t *ota_dat_ptr = (ota_data_t *)&p_rf_rx_data->dat[0];
    if (p_rf_rx_data->rf_len < sizeof(ota_data_t)) {
        return 0;
    }

    ota_data_st *pd = (ota_data_st *)&ota_dat_ptr->dat[0];
    u8 valid = 0;
    if (pd->cmd == CMD_OTA_START || pd->cmd == CMD_OTA_END || pd->cmd == CMD_OTA_VERSION) {
        valid = 1;
    } else {
        unsigned short calc = crc16_user((u8 *)&pd->cmd, 18);
        valid = (calc == pd->crc) ? 1 : 0;
    }

    // DID 过滤（offset 31-34）
    if (valid && d24g_ota_start_flag == 0 && p_rf_rx_data->rf_len >= 35) {
        u32 pkt_target_did = p_rf_rx_data->dat[31] |
                             (p_rf_rx_data->dat[32] << 8) |
                             (p_rf_rx_data->dat[33] << 16) |
                             (p_rf_rx_data->dat[34] << 24);
        if (pkt_target_did != 0 && pkt_target_did != user_var.dongle_id) {
            valid = 0;
        }
    }
    // PID 过滤（offset 27-30，u32）
    if (valid && p_rf_rx_data->rf_len >= 31) {
        u32 pkt_fw_type = p_rf_rx_data->dat[27] |
                          (p_rf_rx_data->dat[28] << 8) |
                          (p_rf_rx_data->dat[29] << 16) |
                          (p_rf_rx_data->dat[30] << 24);
        if (pkt_fw_type != 0 && pkt_fw_type != PID) {
            valid = 0;
        }
    }

    if (valid) {
        if (ota_buff_valid_flag) {
            if (pd->cmd <= ota_start_index) {
                ota_pno_ack_flag = 1;
            }
        } else {
            memcpy(ota_buff.dat, ota_dat_ptr->dat, 20);
            ota_buff_valid_flag = 1;
        }
    }
    return 0;
}

// ============================================================================
// RX / TX IRQ 处理（精简，只处理 OTA）
// ============================================================================
void rf_rx_irq_handle(void)
{
    user_var.raw_pkt = (u8 *)(user_var.rf_rx_buff + user_var.rf_rx_wptr * PKT_BUFF_SIZE);
    user_var.rf_rx_wptr = (user_var.rf_rx_wptr + 1) & 1;
    rf_rx_buffer_set((u8 *)(user_var.rf_rx_buff + user_var.rf_rx_wptr * PKT_BUFF_SIZE), PKT_BUFF_SIZE, 0);
    rf_packet_t *p_rf_rx_data = (rf_packet_t *)user_var.raw_pkt;

    if (RF_TPLL_PACKET_CRC_OK(user_var.raw_pkt) &&
        (RF_TPLL_PACKET_LENGTH_OK(user_var.raw_pkt) ||
         ((p_rf_rx_data->rf_len > 0) && (p_rf_rx_data->rf_len <= 59))))
    {
        if (p_rf_rx_data->dat[0] == CMD_OTA_DATA) {
            rf_ota_data_response(p_rf_rx_data);
        }
    }
    user_var.raw_pkt[0] = 1;
}

void rf_tx_irq_handle(void)
{
    rf_set_tx_rx_off();
    rf_set_channel(rf_chn[user_var.host_channel], 0);
    rf_set_rxmode();
}

void irq_handle_private_1m(void)
{
    unsigned short rf_irq_src = rf_irq_src_get();

    if (rf_irq_src) {
        if (rf_irq_src & FLD_RF_IRQ_RX) {
            rf_irq_clr_src(FLD_RF_IRQ_RX);
            rf_state = RF_RX_END_STATUS;
            rf_rx_irq_handle();
        }
        if (rf_irq_src & FLD_RF_IRQ_TX) {
            rf_irq_clr_src(FLD_RF_IRQ_TX);
            if (ota_auto_tx_pending) {
                ota_auto_tx_pending = 0;
            } else {
                rf_tx_irq_handle();
            }
            rf_state = RF_IDLE_STATUS;
        }
        if (rf_irq_src & FLD_RF_IRQ_RX_TIMEOUT) {
            rf_irq_clr_src(FLD_RF_IRQ_RX_TIMEOUT);
            rf_state = RF_RX_TIMEOUT_STATUS;
        }
    }
}

// 读取 app 跳转 bootloader 前写入的版本号（APP_VERSION_ADDR）。
// 有 app（写入过）返回 app 版本；无 app（未写入/0xFF）返回 0。
static u32 boot_get_fw_version(void)
{
    u32 ver = 0;
    flash_read_page(APP_VERSION_ADDR, 4, (u8 *)&ver);
    if (ver == 0xFFFFFFFF) {
        ver = 0;
    }
    return ver;
}

// ============================================================================
// OTA poll 发送（精简自 ota_probe_send_poll）
// ============================================================================
void ota_probe_send_poll(void)
{
    static u32 poll_tick = 0;
    // END 收尾：pno=1 已随 poll 发出后不再发，等主循环搬运重启
    if (ota_end_ack_sent) {
        return;
    }
    u32 interval_us = (d24g_ota_status == 1) ? 3000 : 100000;
    // END 待确认时强制立即发（不等轮询间隔），确保 pno=1 尽快到 dongle
    if (!clock_time_exceed(poll_tick, interval_us) && !ota_end_done) {
        return;
    }
    poll_tick = clock_time();

    // 等待 START 阶段：每 1s 打印一次 poll 状态，确认 poll 在持续发送
    if (d24g_ota_start_flag == 0) {
        static u32 poll_log_tick = 0;
        if (clock_time_exceed(poll_log_tick, 1000000)) {
            poll_log_tick = clock_time();
            TOY_LOG("---boot_poll: wait START, did=0x%08x tick=%d\n",
                   user_var.dongle_id, clock_time());
        }
    }

    rf_packet_t *tx = &user_var.ack_rf_dat;
    tx->dma_len = 20;
    tx->rf_len  = 19;
    tx->dat[0] = CMD_OTA_DATA;
    tx->dat[1] = 0;
    tx->dat[2] = ota_pno_ack_flag ? 1 : 0;
    ota_pno_ack_flag = 0;
    tx->dat[3] = (u8)(user_var.dongle_id & 0xff);
    tx->dat[4] = (u8)((user_var.dongle_id >> 8) & 0xff);
    tx->dat[5] = (u8)((user_var.dongle_id >> 16) & 0xff);
    tx->dat[6] = (u8)((user_var.dongle_id >> 24) & 0xff);
    // PID(u32) at dat[7..10], FW_VERSION(u32) at dat[11..14], expect_idx(u16) at dat[15..16]
    memset(&tx->dat[7], 0, 10);
    tx->dat[7]  = (u8)((PID >> 0)  & 0xff);  // PID u32 小端，供 dongle 按产品过滤 OTA
    tx->dat[8]  = (u8)((PID >> 8)  & 0xff);
    tx->dat[9]  = (u8)((PID >> 16) & 0xff);
    tx->dat[10] = (u8)((PID >> 24) & 0xff);
    // 上报版本 = app 跳转时存入的版本（无 app 则 0），而非编译期宏
    u32 fw_ver = boot_get_fw_version();
    tx->dat[11] = (u8)((fw_ver >> 0)  & 0xff);
    tx->dat[12] = (u8)((fw_ver >> 8)  & 0xff);
    tx->dat[13] = (u8)((fw_ver >> 16) & 0xff);
    tx->dat[14] = (u8)((fw_ver >> 24) & 0xff);
    // 上报设备期望的下一帧 DATA 序号（dat[15]=低字节，dat[16]=高字节），
    // dongle 据此选择性重传。必须在 memset(&dat[7],0,10) 之后设置，避免被清零。
    tx->dat[15] = (u8)(ota_expect_idx & 0xff);
    tx->dat[16] = (u8)((ota_expect_idx >> 8) & 0xff);
    u16 crc = crc16_user(&tx->dat[0], 17);
    tx->dat[17] = (u8)(crc & 0xff);
    tx->dat[18] = (u8)((crc >> 8) & 0xff);

    device_channel = user_var.host_channel;
    rf_tx_acc_code_select(0);
    rf_set_pair_access_code(pair_var.pair_accesscode);

    ota_auto_tx_pending = 1;
    if (ota_end_done) {
        ota_end_ack_sent = 1;   // 本次 poll 即 END 确认（pno=1），发完等 TX 完成即重启
    }
    rf_stx_to_rx((u8 *)tx, 2000);
}

// ============================================================================
// 判断 OTA 区是否有完整固件（接收完成待搬运 或 搬运中断）
// ============================================================================
static u8 boot_ota_ready(void)
{
    u32 fw_size = 0;
    flash_read_page(OTA_ADDRESS + 0x18, 4, (u8 *)&fw_size);
    if (fw_size >= OTA_SIZE * 1024) {
        TOY_LOG("---boot_ota_ready: ota fw_size=%d >= %d, not ready\n", fw_size, OTA_SIZE * 1024);
        return 0;
    }
    u8 ok = (flash_fw_check(0xffffffff, OTA_ADDRESS) == 0);
    TOY_LOG("---boot_ota_ready: ota fw_size=%d crc_ok=%d\n", fw_size, ok);
    return ok;
}

// ============================================================================
// 搬运 OTA 区固件到 app 区（校验 + 逐块拷贝 + 清 OTA 首扇区）
// ============================================================================
static u8 boot_copy_fw_from_ota(void)
{
    u32 fw_size = 0;
    flash_read_page(OTA_ADDRESS + 0x18, 4, (u8 *)&fw_size);
    TOY_LOG("---boot_copy: ota fw_size=%d\n", fw_size);

    // 关键保护：OTA 区为垃圾/空（0xFF 等）时 fw_size 会异常巨大，
    // 若直接进 flash_fw_check 会按其计算循环 65535 次（约 17s），
    // 远超看门狗窗口导致复位死循环。这里先校验合法性。
    if ((fw_size == 0) || (fw_size >= OTA_SIZE * 1024)) {
        TOY_LOG("---boot_copy: ota fw_size=%d invalid, erase OTA\n", fw_size);
        flash_erase_sector(OTA_ADDRESS);
        return 0;
    }

    if (flash_fw_check(0xffffffff, OTA_ADDRESS) != 0) {
        // 整包 CRC 校验失败：擦除 OTA 区，避免被误拷贝
        TOY_LOG("---boot_copy: OTA CRC fail, erase OTA\n");
        flash_erase_sector(OTA_ADDRESS);
        return 0;
    }

    hal_flash_unlock();
    u8  buf[256];
    u32 n = fw_size / 256;
    if (fw_size % 256) n++;
    TOY_LOG("---boot_copy: start copy %d blocks (OTA 0x%06x -> FW 0x%06x)\n",
           n, OTA_ADDRESS, FW_ADDRESS);

    for (u32 i = 0; i < n; i++) {
        if ((FW_ADDRESS + i * 256) % 4096 == 0) {
            flash_erase_sector(FW_ADDRESS + i * 256);
        }
        flash_read_page(OTA_ADDRESS + i * 256, 256, buf);
        flash_write_page(FW_ADDRESS + i * 256, 256, buf);
        if ((i % 64) == 0) {
            TOY_LOG("---boot_copy: %d/%d\n", i, n);
        }
    }
    // 清 OTA 首扇区（清 startup_flag，防止下次 boot 重复拷贝）
    flash_erase_sector(OTA_ADDRESS);
    TOY_LOG("---boot_copy: done, OTA sector cleared\n");
    return 1;
}

// ============================================================================
// OTA 结果处理（bootloader 版：成功搬运重启，失败清标记重启）
// ============================================================================
void d24g_ota_resut(u8 result)
{
    TOY_LOG("---boot_ota_resut: result=%d tick=%d\n", result, clock_time());
    if (result == OTA_SUCCESS) {
        ota_flag_clear();          // 先清标记：断电窗口(标记清+OTA完整)重新上电会继续搬运
        boot_copy_fw_from_ota();   // 校验 + 搬运 OTA -> app（内部清 OTA 首扇区）
    } else {
        flash_erase_sector(OTA_ADDRESS);  // 擦半成品
        ota_flag_clear();          // 清标记，超时后回到旧固件
    }
    rf_set_tx_rx_off();
    reg_dma_chn_en &= ~(FLD_DMA_CHN_RF_RX | FLD_DMA_CHN_RF_TX);
    for (volatile int i = 0; i < 100; i++) {}
    start_reboot();
}

// ============================================================================
// OTA 写状态机（精简自 d24g_ota_write；bootloader 上电即 OTA 模式）
// ============================================================================
static u8 ota_boot_write(ota_buff_t *p_usb)
{
    ota_data_st *pd = (ota_data_st *)&p_usb->dat[0];

    // VERSION：bootloader 已决定要 OTA，接受匹配 PID 的查询并通知 dongle 就绪
    if (pd->cmd == CMD_OTA_VERSION) {
        ota_version_query_t *vq = (ota_version_query_t *)pd->buf;
        if (vq->target_pid != 0 && vq->target_pid != PID) {
            TOY_LOG("---boot_ota: VERSION pid=0x%08x != self 0x%08x, ignore\n",
                    vq->target_pid, (u32)PID);
            return OTA_SUCCESS;
        }
        ota_pending = 1;
        ota_pno_ack_flag = 1;
        TOY_LOG("---boot_ota: VERSION recv tick=%d\n", clock_time());
        return OTA_SUCCESS;
    }

    if (pd->cmd == CMD_OTA_START) {
        if (ota_pending == 0) {
            TOY_LOG("---boot_ota: START without VERSION, ignore\n");
            return OTA_SUCCESS;
        }
        if (d24g_ota_start_flag) {
            ota_pno_ack_flag = 1; // 重复 START，re-ack
            TOY_LOG("---boot_ota: dup START, re-ack\n");
            return OTA_SUCCESS;
        }
        d24g_ota_status = 1;
        d24g_ota_start_flag = 1;
        blt_ota_start_tick = clock_time();
        TOY_LOG("---boot_ota: START recv tick=%d, erase OTA sector\n", blt_ota_start_tick);
        ota_program_offset = OTA_ADDRESS;
        ota_start_index = 0;
        ota_expect_idx = 0;   // 期望第一个 DATA 包 #0
        ota_flash_write_addr = 0;
        ota_fw_size = 0;
        ota_error_flag = OTA_SUCCESS;

        hal_flash_unlock();
        flash_erase_sector(ota_program_offset);

        ota_pno_ack_flag = 1;
        return ota_error_flag;
    }

    if ((ota_error_flag == OTA_SUCCESS) && d24g_ota_start_flag) {
        blt_ota_start_tick = clock_time();

        if (pd->cmd == CMD_OTA_END) {
            ota_pno_ack_flag = 1;
            TOY_LOG("---boot_ota: END recv, fw_size=%d start_idx=%d\n", ota_fw_size, ota_start_index);

            u32 *telink_mark = (u32 *)&ota_first_data_buf[8];
            if (telink_mark[0] != START_UP_FLAG) {
                TOY_LOG("---boot_ota: END mark err 0x%08x\n", telink_mark[0]);
                ota_error_flag = OTA_FIRMWARE_MARK_ERR;
                return ota_error_flag;
            }

            u32 real_bin_size = ota_fw_size;
            if ((real_bin_size % 16) == 0) {
                if (real_bin_size != ((ota_start_index + 1) * 16)) {
                    ota_error_flag = OTA_FW_SIZE_ERR;
                    return ota_error_flag;
                }
            } else {
                if ((real_bin_size - (real_bin_size % 16)) != (ota_start_index * 16)) {
                    ota_error_flag = OTA_FW_SIZE_ERR;
                    return ota_error_flag;
                }
            }

            flash_write_page(ota_program_offset, 16, ota_first_data_buf);

            u8 read_flash_buf[16];
            flash_read_page(ota_program_offset, 16, read_flash_buf);
            if (memcmp(read_flash_buf, ota_first_data_buf, 16)) {
                flash_erase_sector(ota_program_offset);
                ota_error_flag = OTA_WRITE_FLASH_ERR;
                return ota_error_flag;
            }

            // END 校验通过：不在此直接 d24g_ota_resut() 重启，否则 pno=1 的 poll
            // 永远发不出去，dongle 会 10s END ack timeout 才 re-broadcast 下一设备。
            // 改为主循环发 pno=1 确认 END、等 TX 完成后再搬运固件并重启。
            ota_end_done = 1;
            return OTA_SUCCESS;
        } else {
            if (pd->cmd == CMD_OTA_VERSION) {
                return OTA_SUCCESS;
            }

            u16 calc_crc = crc16_user((u8 *)&pd->cmd, 18);
            if (calc_crc == pd->crc) {
                if (pd->cmd == 0x0000) {
                    if (ota_flash_write_addr > 0) {
                        ota_pno_ack_flag = 1;
                        return OTA_SUCCESS;
                    }
                    memcpy(ota_first_data_buf, pd->buf, 16);
                    ota_start_index = 0;
                    ota_pno_ack_flag = 1;
                } else {
                    if ((ota_start_index + 1) != pd->cmd) {
                        if (pd->cmd <= ota_start_index) {
                            ota_pno_ack_flag = 1;
                            return OTA_SUCCESS;
                        }
                        return OTA_SUCCESS;  // 超前包，丢弃
                    }
                    ota_start_index = pd->cmd;
                    ota_pno_ack_flag = 1;

                    if (pd->cmd == 0x0001) {
                        ota_fw_size = pd->buf[8] | (pd->buf[9] << 8) |
                                      (pd->buf[10] << 16) | (pd->buf[11] << 24);
                        TOY_LOG("---boot_ota: pkt#1 fw_size=%d\n", ota_fw_size);
                        if (ota_fw_size > (OTA_SIZE * 1024)) {
                            ota_error_flag = OTA_FW_SIZE_ERR;
                            return ota_error_flag;
                        }
                    }

                    if (ota_fw_size < ota_flash_write_addr) {
                        ota_error_flag = OTA_FW_SIZE_ERR;
                        return ota_error_flag;
                    }

                    if ((ota_flash_write_addr % 4096) == 0) {
                        flash_erase_sector(ota_flash_write_addr + ota_program_offset);
                    }
                    flash_write_page(ota_flash_write_addr + ota_program_offset, 16, pd->buf);
                }
                ota_flash_write_addr += 16;
                ota_expect_idx = (u16)(ota_start_index + 1);  // 已写入 ota_start_index 帧，期望下一帧
                // DATA 进度：每 500 包打印一次，避免刷屏
                if ((ota_start_index % 500) == 0) {
                    TOY_LOG("---boot_ota: pkt#%d fwa=0x%x\n", ota_start_index, ota_flash_write_addr);
                }
            } else {
                // 内层 CRC 校验失败：丢弃本包、不推进序号、不 ack。
                // 设备 poll 的 expected_idx 停留本帧（不置 ota_pno_ack_flag），
                // dongle 据此重发该帧，而非直接终止 OTA。
                TOY_LOG("---boot_ota: DATA CRC err pkt#%d, drop & wait retransmit\n", pd->cmd);
                return OTA_SUCCESS;
            }
        }
    }

    return ota_error_flag;
}

// ============================================================================
// OTA 超时看门狗
// ============================================================================
void d24g_ota_loop(void)
{
    if (d24g_ota_start_flag) {
        // START 之后：数据接收阶段 60s 超时
        if (clock_time_exceed(blt_ota_start_tick, D24G_OTA_TIMEOUT_US)) {
            d24g_ota_resut(OTA_TIMEOUT);
        }
    } else {
        // 等待 START 阶段：dongle 失联/取消/串行排队过久时，超时清标记重启回 app，
        // 避免设备（含断电重开）永久卡死在 bootloader 的 poll 循环。
        if (clock_time_exceed(blt_ota_start_tick, D24G_OTA_WAIT_START_TIMEOUT_US)) {
            d24g_ota_resut(OTA_TIMEOUT);
        }
    }
}

// ============================================================================
// bootloader 的 OTA 主流程：RF 初始化 + poll 扫描 dongle + 接收固件
// ============================================================================
void boot_ota_run(void)
{
    d24g_ota_status = 1;
    d24g_ota_start_flag = 0;
    ota_program_offset = OTA_ADDRESS;
    ota_error_flag = OTA_SUCCESS;
    ota_expect_idx = 0;   // 等 START 阶段 poll 上报 0（冷启动 retention 可能非 0）
    ota_end_done = 0;
    ota_end_ack_sent = 0;
    blt_ota_start_tick = clock_time();
    TOY_LOG("---boot_ota_run: enter tick=%d\n", blt_ota_start_tick);

    custom_init();
    TOY_LOG("---boot_ota_run: custom_init done, did=0x%08x\n", user_var.dongle_id);
    rf_drv_private_1m_init();
    d24_user_init();
    TOY_LOG("---boot_ota_run: RF init + RX done, start poll loop\n");

    while (1) {
#if (MODULE_32K_WATCHDOG_ENABLE)
        wd_32k_feed();  // OTA 接收可能持续数分钟，喂狗防止看门狗复位
#endif
        // 24M RC 定时校准：每 1s 一次，保持 RF 频率精度（参考 Robin_24G_Demo）
        static u32 rc_cal_tick = 0;
        if (clock_time_exceed(rc_cal_tick, 1000000)) {
            rc_cal_tick = clock_time();
            rc_24m_cal();
        }

        d24g_ota_loop();
        if (ota_buff_valid_flag) {
            u8 err = ota_boot_write(&ota_buff);
            if (err != OTA_SUCCESS) {
                d24g_ota_resut(err);
            }
            ota_buff_valid_flag = 0;
        }
        ota_probe_send_poll();
        // END 已校验通过且 pno=1 已随 poll 发出（TX 完成）：搬运固件并重启。
        // 必须等 TX 完成，否则 pno=1 未到 dongle 就重启，dongle 会 10s 超时兜底。
        if (ota_end_done && ota_end_ack_sent && (ota_auto_tx_pending == 0)) {
            d24g_ota_resut(OTA_SUCCESS);
        }
    }
}

// ============================================================================
// bootloader OTA 入口决策（main 调用，替代直接 boot_ota_run）：
//   OTA 区有完整固件（接收完成或搬运中断）-> 重新搬运（幂等）+ 清标记 + 重启。
//   否则进 OTA 接收模式。
//   boot_need_ota() 已保证：标记在 / app 无效 / OTA 区完整 三者至少其一。
// ============================================================================
void boot_ota_entry(void)
{
    TOY_LOG("---boot_ota_entry: try copy OTA->app\n");
    if (boot_copy_fw_from_ota()) {
        ota_flag_clear();
        TOY_LOG("---boot_ota_entry: copied OTA->app, reboot\n");
        start_reboot();  // 重启，下次 boot 跳 app（不返回）
    }
    boot_ota_run();  // 进 OTA 接收模式（不返回）
}

// ============================================================================
// 判断 app 区是否有有效固件
// ============================================================================
u8 boot_app_is_valid(void)
{
    u32 startup_flag = 0;
    u32 fw_size = 0;
    flash_read_page(FW_ADDRESS + 0x8, 4, (u8 *)&startup_flag);
    flash_read_page(FW_ADDRESS + 0x18, 4, (u8 *)&fw_size);
    if ((startup_flag == START_UP_FLAG) && (fw_size < OTA_SIZE * 1024)) {
        u8 ok = (flash_fw_check(0xffffffff, FW_ADDRESS) == 0);
        TOY_LOG("---boot_app_valid: app flag=0x%08x fw_size=%d crc_ok=%d\n",
               startup_flag, fw_size, ok);
        if (ok) {
            return 1;
        }
    } else {
        TOY_LOG("---boot_app_valid: app flag=0x%08x(exp 0x%08x) fw_size=%d(exp<%d) -> invalid\n",
               startup_flag, START_UP_FLAG, fw_size, OTA_SIZE * 1024);
    }
    return 0;
}
