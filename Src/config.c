#include "config.h"

// 这个是机器编号，只能是一个字节。1~255.
// 不能使用 0 作为 id，那是一个广播编号。
uint16_t machine_id = 128;

// 移动到比例位置的时候使用的步数范围。
int32_t lm_conf_limit_out = -110000;
int32_t lm_conf_limit_in = -5000;

// 电机循环测试
int lm_cycle_out = -100000;
int lm_cycle_in = -5000;
// 1的时候只有跑完一整个测试循环，电机在最里面的位置才会暂停一会儿，
// 0的时候在测试里的每一步都会暂停一会儿。
int lm_cycle_pause_at_full_cycle = 1;
int lm_cycle_step_pause = 500;

// WiFi 参数
char wifi_conf_ssid[WIFI_STRSIZE] = "ssid";
char wifi_conf_pwd[WIFI_STRSIZE] = "pwd";
char wifi_conf_tcpip[WIFI_STRSIZE] = "192.168.0.103";
uint16_t wifi_conf_tcpport = 5577;
