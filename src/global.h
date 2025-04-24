#ifndef GLOBAL_H
#define GLOBAL_H

#include <cstdint> // 包含 uint16_t 的定义

// 定义全局变量
extern uint16_t globalPn;
extern uint16_t globalSn;
extern uint8_t globalstatus;
extern uint16_t globalangle;
extern uint8_t globalspeed;
extern uint8_t globalminrange;
extern uint8_t globalmaxrange;
extern int globalx;
extern int globalcount;
extern int count_0_9999;
extern int count_10000_19999;
extern int count_20000_29999;
extern int count_30000_39999;
extern int count_40000_49999;
extern int count_50000_59999;
extern int count_60000_65535;
extern int Accumulate;
extern int thereold;
extern char sendBuffer[256];
// 标志位
extern int flag_shanxing;   // 标志位 1: 启用扇形处理, 0: 禁用
extern int flag_target;     // 标志位 1: 检测到目标, 0: 未检测到
extern int flag_zhiling;    // 标志位 1: 执行某指令, 0: 未执行指令
extern int flag_jingzhi;    // 标志位进入运动静止判别 1: 进入静止, 0: 正在运动
extern int flag_yundong;	// 标志位 1: 正在运动, 0: 静止
extern int flag_beijing;	// 标志位 1: 与背景比有目标, 0: 与背景比无目标
//计数器
extern int jishu_shanxing;
extern int jishu_guding;
extern int jishu_mubiao;
extern int jishi_mubiao;
#endif // GLOBAL_H