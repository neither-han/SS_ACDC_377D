#include "digital_filter.h"
#include <float.h>
#include <stddef.h>

static float DF_Clamp(float value, float lower, float upper)
{
    if (value > upper) { return upper; }
    if (value < lower) { return lower; }
    return value;
}

int DF_Init(DigitalFilter *filter, uint16_t num_stages,
            const DF_Coefficients *coeff, DF_State *state,
            const DF_OutputLimits *limits)
{
    DigitalFilter candidate;
    if (filter == NULL ||
        (num_stages != 0U && (coeff == NULL || state == NULL))) {
        return 0;
    }
    /* 先构造并验证临时配置；失败时不改变旧实例或清零状态。 */
    candidate.num_stages = num_stages;
    candidate.coeff = coeff;
    candidate.state = state;
    candidate.limits = NULL;
    if (!DF_SetStageOutputLimits(&candidate, limits)) {
        return 0;
    }
    *filter = candidate;
    DF_Reset(filter);
    return 1;
}

void DF_Reset(DigitalFilter *filter)
{
    uint16_t i;
    for (i = 0U; i < filter->num_stages; ++i) {
        filter->state[i].s1 = 0.0f;
        filter->state[i].s2 = 0.0f;
    }
}

int DF_SetStageOutputLimits(DigitalFilter *filter,
                            const DF_OutputLimits *limits)
{
    uint16_t i;
    if (filter == NULL ||
        (filter->num_stages != 0U &&
         (filter->coeff == NULL || filter->state == NULL))) {
        return 0;
    }
    if (limits != NULL) {
        for (i = 0U; i < filter->num_stages; ++i) {
            const float lower = limits[i].min;
            const float upper = limits[i].max;

            /* 有限性与上下限顺序统一检查：NaN 不满足有序比较，
             * 正负无穷超出 FLT_MAX 范围。仅在配置时执行。
             */
            if (!(lower >= -FLT_MAX && lower <= FLT_MAX &&
                  upper >= -FLT_MAX && upper <= FLT_MAX &&
                  lower <= upper)) {
                return 0;
            }
        }
    }
    /* 全部节验证成功后才提交，失败不会留下部分配置。 */
    filter->limits = limits;
    return 1;
}

float DF_Process(DigitalFilter *filter, float input)
{
    uint16_t i;
    float x = input;
    const DF_Coefficients *c = filter->coeff;
    DF_State *s = filter->state;

    for (i = 0U; i < filter->num_stages; ++i) {
        const float raw = c->b0 * x + s->s1;

        /* 原始二阶递推：使用未限幅输出更新状态。
         * 先合并反馈项和旧 s2，再加较小的前馈项，减少小量被吞掉。
         */
        s->s1 = (s->s2 - c->a1 * raw) + c->b1 * x;
        s->s2 = c->b2 * x - c->a2 * raw;

        /* 仅限制本节对外输出，再送入下一级。 */
        x = raw;
        if (filter->limits != NULL) {
            x = DF_Clamp(raw, filter->limits[i].min, filter->limits[i].max);
        }
        ++c;
        ++s;
    }
    return x;
}
