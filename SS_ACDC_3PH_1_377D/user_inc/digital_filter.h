#ifndef DIGITAL_FILTER_H
#define DIGITAL_FILTER_H

#include <stdint.h>

/* 每节 H(z)=(b0+b1*z^-1+b2*z^-2)/(1+a1*z^-1+a2*z^-2)。
 * 导入前将全部系数除以原始 a0；a1/a2 保留分母符号，不额外取反。
 * a1=a2=0 可实现 FIR；b2=a2=0 可表示一阶节。
 */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
} DF_Coefficients;

typedef struct {
    float s1, s2;
} DF_State;

/* 每个二阶节对应一组限值，单位为该节输出的单位。 */
typedef struct {
    float min;
    float max;
} DF_OutputLimits;

typedef struct {
    uint16_t num_stages;       /* 级联节数，不是总阶数。 */
    const DF_Coefficients *coeff;
    DF_State *state;
    const DF_OutputLimits *limits; /* NULL：不限幅，否则每节一组。 */
} DigitalFilter;

/**
 * @brief 一次完成系数绑定、逐节输出限幅配置和状态清零。
 * @param filter 待初始化的非 NULL 实例。
 * @param num_stages 级联节数；0 表示直通。
 * @param coeff 已归一化的系数数组，至少 num_stages 个元素。
 * @param state 可写状态数组，至少 num_stages 个元素。
 * @param limits 每节一组限值，至少 num_stages 个元素；NULL 表示不限幅。
 * @return 1：成功；0：filter 为 NULL，非零节数时 coeff/state 为 NULL，
 *         或任一节限值非有限/下限大于上限；
 *         失败时原实例及状态数组保持不变。
 * @details 内部调用 DF_SetStageOutputLimits 验证和绑定限幅配置；
 *          全部检查成功后才保存实例并调用 DF_Reset 清零状态。
 *          调用者无需再单独调用限幅配置函数。
 * @pre 系数必须有限且经过设计验证；数组在实例使用期间保持有效。
 * @note 0 节时 coeff/state 可为 NULL。独立实例可以共享系数，
 *       但不能共享/重叠状态；系数和状态也不能重叠。
 *       limits 只保存指针、不复制，使用期间须保持有效且不可修改，
 *       不能与可写状态重叠；0 节时不读取限值数组，直接透传输入。
 *       本函数不检查数组实际长度、系数有限性或滤波器稳定性。
 *       开启中断前调用，运行中重新初始化须暂停对应实例的处理。
 */
int DF_Init(DigitalFilter *filter, uint16_t num_stages,
            const DF_Coefficients *coeff, DF_State *state,
            const DF_OutputLimits *limits);

/**
 * @brief 清零全部状态，保留系数、节数及限幅配置。
 * @param filter 已成功初始化的非 NULL 实例。
 * @note 即使限幅区间不包含 0，状态仍清零；下次计算才限制输出。
 *       不得与同一实例的 DF_Process 并发调用。
 */
void DF_Reset(DigitalFilter *filter);

/**
 * @brief 供 DF_Init 配置限幅，也可用于初始化后更换或关闭限幅。
 * @param[in,out] filter 已初始化实例，或 DF_Init 内已绑定节数/系数/状态
 *                      的临时实例；本函数不读取状态数组内容。
 * @param[in] limits 至少 num_stages 个元素，每节独立设置 min/max；
 *               NULL 表示关闭逐节输出限幅。
 * @return 1：全部检查通过并保存配置；0：filter 为 NULL，非零节数时
 *         coeff/state 为 NULL，或任一节限值非有限/下限大于上限。
 *         失败时原配置及全部状态保持不变。
 * @details 在本函数内检查每组限值：拒绝 NaN、正负无穷及 min > max，
 *          允许 min == max（该节对外输出固定为此值）。
 *          0 节时不读取限值数组，保存指针但不执行输出限幅。
 *          数组仅保存指针、不复制，配置期间必须持续有效且不可修改，
 *          也不能与可写状态重叠。更换配置不重置状态，下次处理时生效。
 * @pre 外部单独调用时须先成功调用 DF_Init；DF_Init 内部调用时
 *      已先设置临时实例的全部字段。本函数不能识别未初始化的实例，
 *      也不能检查非 NULL 指针的实际有效性或数组实际长度。
 * @note 仅限制各节对外输出；状态递推仍使用该节未限幅输出。
 *       对应结构：二阶传递函数 -> 限幅 -> 下一个二阶传递函数。
 *       限幅不保证内部状态有界，系数及内部动态范围仍需合理设计。
 *       调用时必须暂停对应实例的处理。
 */
int DF_SetStageOutputLimits(DigitalFilter *filter,
                            const DF_OutputLimits *limits);

/**
 * @brief 处理一个采样点，执行全部二阶节，更新状态并逐节限制输出。
 * @param[in,out] filter 已成功初始化的非 NULL 滤波器实例。
 * @param[in] input 当前采样点，必须为有限 float 数值。
 * @return 最后一级的输出；num_stages 为 0 时原样返回 input。
 * @details 各节依次执行：
 *          raw = b0*x + s1;
 *          s1 = (s2 - a1*raw) + b1*x;
 *          s2 = b2*x - a2*raw;
 *          y = clamp(raw, limits[i].min, limits[i].max); // 若配置限幅
 *          x = y; // 作为下一级本次输入
 *          状态跨调用保存。限幅只作用于输出，不直接限制/修正状态。
 *          未触发限幅时，各节遵循系数定义的线性传递函数。
 * @pre 系数、输入和内部动态范围应保证浮点计算不会溢出。
 * @note 在 25 kHz 中断中，每个实例每个采样点调用一次，Ts=40 us。
 *       不得每次调用前初始化或清零。无动态内存和除法，
 *       每节基本计算为 5 次乘法、4 次加减法，以及可选的限幅比较。
 *       为减少实时开销，本函数不检查指针、数组长度或 NaN。
 *       同一实例不可重入，也不能同时重置、更换系数或配置限幅。
 *       复制 DigitalFilter 只会复制指针，不会产生独立状态。
 *       实现位于 digital_filter.c，必须将其加入 DSP 工程编译。
 */
float DF_Process(DigitalFilter *filter, float input);

#endif
