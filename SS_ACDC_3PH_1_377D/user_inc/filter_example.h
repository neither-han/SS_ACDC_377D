#ifndef FILTER_EXAMPLE_H
#define FILTER_EXAMPLE_H

#include "digital_filter.h"

/* 两个独立的二阶 Butterworth 低通，采样频率均为 25000 Hz。
 * 公共只读系数可复用；每个新通道必须另外分配状态。
 */
extern const DF_Coefficients LPF10Hz_Coefficients[1];
extern const DF_Coefficients LPF1kHz_Coefficients[1];

/**
 * @brief 初始化 10 Hz 二阶低通例程，返回 1 成功、0 失败。
 * @note 开启中断前调用。示例输出范围 [-1,1]，在 .c 中修改限值。
 */
int LPF10Hz_Init(void);

/**
 * @brief 在 25 kHz 中断中处理一个采样点，返回 10 Hz 低通的限幅输出。
 * @param input 当前采样值；必须先成功调用 LPF10Hz_Init。
 * @return 当前采样点的滤波结果，限制在例程配置的输出范围内。
 * @note input 必须有限；同一例程不可并发调用或同时重新初始化。
 */
float LPF10Hz_Process(float input);

/**
 * @brief 初始化 1 kHz 二阶低通例程，返回 1 成功、0 失败。
 * @note 开启中断前调用。示例输出范围 [-1,1]，在 .c 中修改限值。
 */
int LPF1kHz_Init(void);

/**
 * @brief 在 25 kHz 中断中处理一个采样点，返回 1 kHz 低通的限幅输出。
 * @param input 当前采样值；必须先成功调用 LPF1kHz_Init。
 * @return 当前采样点的滤波结果，限制在例程配置的输出范围内。
 * @note input 必须有限；同一例程不可并发调用或同时重新初始化。
 */
float LPF1kHz_Process(float input);

#endif
