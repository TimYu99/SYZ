// global.cpp

#include "global.h"

// 全局变量定义
uint16_t globalPn = 0;
uint16_t globalSn = 0;
uint8_t globalstatus1 = 0x00;
uint8_t globalstatus2 = 0x00;
uint16_t globalangle = 0;
uint8_t globalspeed = 0;
uint8_t globalminrange = 0;
uint8_t globalmaxrange = 0;
int globalcount = 0;
int globalx = 0;
int count_0_9999;
int count_10000_19999;
int count_20000_29999;
int count_30000_39999;
int count_40000_49999;
int count_50000_59999;
int count_60000_65535;

int Accumulate;
int thereold;
char sendBuffer[256] = "";
char sendBuffer2[256] = "";
// 全局变量定义目标判别时的标志和初始化
int flag_jingzhi = 0;   // 默认关闭
int flag_yundong = 1;   // 默认关闭
int flag_shanxing = 0;   // 默认关闭
int flag_target = 0;     // 默认未检测到目标
int flag_zhiling = 1;    // 默认未执行指令
int flag_beijing = 0;    // 默认未执行指令
int smsn1_on_flag = 0;
int smsn2_on_flag = 0;
//计数器
int jishu_shanxing =0;
int jishu_guding=0;
int jishu_mubiao = 0;
int jishi_mubiao = 0;