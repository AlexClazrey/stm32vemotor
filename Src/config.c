#include "config.h"

// 这个是机器编号，只能是一个字节。
uint8_t machine_id = 124;

// 移动到比例位置的时候使用的范围。
const int lm_limit_out = -110000;
const int lm_limit_in = -5000;

// 电机循环测试
const int lm_cycle_out = -10000;
const int lm_cycle_in = -5000;
// 1的时候只有跑完一整个测试循环，点击在最里面的位置才会暂停一会儿，
// 0的时候在测试里的每一步都会暂停一会儿。
int lm_cycle_pause_at_full_cycle = 1;

// CAN 命令参数
const uint8_t CAN_CMD_VER = 2;
const uint8_t CAN_CMD_LM = 1;
const uint8_t CAN_CMD_STRING = 10;

// 主循环时间设置
const uint32_t COUNT_INTV = 10; // 每次主循环使用的毫秒数
const uint32_t COUNT_LIMIT = 1000; // COUNT_LIMIT * COUNT_INTV / 1000 得到一个大循环使用的秒数，现在是100s。

char* WIFI_SSID = "SSID";
char* WIFI_PWD = "password";
char* WIFI_TCP_IP = "192.168.1.100";
uint16_t WIFI_TCP_PORT = 5577;
