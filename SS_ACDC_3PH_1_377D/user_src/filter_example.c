#include "filter_example.h"

/* 二阶 Butterworth 低通，Fs=25000 Hz，双线性变换并预畸变截止频率。
 * K=tan(pi*fc/Fs)，D=1+sqrt(2)*K+K*K：
 * b0=K*K/D，b1=2*b0，b2=b0，
 * a1=2*(K*K-1)/D，a2=(1-sqrt(2)*K+K*K)/D。
 * 系数离线生成，中断内不计算三角函数。
 *
 * 10 Hz 相对采样率很低，float 系数量化较敏感：
 * 以下系数量化后的理论直流增益约为 0.9979813；
 * 实际 float 递推还存在舍入误差，精度需求需结合输入幅值评估。
 * 当前主机 GCC -O2 测试中，恒定输入 0.5f，运行 50000 点后输出
 * 约为 0.4953508f（相对输入约 -0.93%），10 Hz 正弦增益约 -3.01093 dB。
 * 该结果不是 TI 编译器/目标 DSP 的误差保证，目标机须另行验证。
 */
const DF_Coefficients LPF10Hz_Coefficients[1] = {
    {1.576334488059e-06f, 3.152668976117e-06f, 1.576334488059e-06f,
     -1.996445697381e+00f, 9.964520027193e-01f}
};

const DF_Coefficients LPF1kHz_Coefficients[1] = {
    {1.335920002786e-02f, 2.671840005571e-02f, 1.335920002786e-02f,
     -1.647459981077e+00f, 7.008967811884e-01f}
};

/* 两个例程的限值相互独立。[-1,1000] 仅为示例，应按采样信号单位修改。
 * 可调用 DF_SetStageOutputLimits(&filter, NULL) 关闭该实例的逐节限幅。
 */
static const DF_OutputLimits lpf10_limits[1] = {{-1.0f, 1000.0f}};
static const DF_OutputLimits lpf1k_limits[1] = {{-1.0f, 1000.0f}};
static DF_State lpf10_state[1];
static DF_State lpf1k_state[1];
static DigitalFilter lpf10_filter;
static DigitalFilter lpf1k_filter;

int LPF10Hz_Init(void)
{
    return DF_Init(&lpf10_filter, 1U, LPF10Hz_Coefficients,
                   lpf10_state, lpf10_limits);
}

float LPF10Hz_Process(float input)
{
    return DF_Process(&lpf10_filter, input);
}

int LPF1kHz_Init(void)
{
    return DF_Init(&lpf1k_filter, 1U, LPF1kHz_Coefficients,
                   lpf1k_state, lpf1k_limits);
}

float LPF1kHz_Process(float input)
{
    return DF_Process(&lpf1k_filter, input);
}
