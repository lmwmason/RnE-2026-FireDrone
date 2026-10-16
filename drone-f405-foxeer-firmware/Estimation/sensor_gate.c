#include "sensor_gate.h"
#include "../Math/matrix.h"

bool sensor_gate_check(const float *v, const float *S_inv, uint8_t n, float threshold)
{
    float S_inv_v[6];
    mat_mul(S_inv, v, S_inv_v, n, n, 1);

    float mahal_sq = 0.0f;
    for (uint8_t i = 0; i < n; i++) mahal_sq += v[i] * S_inv_v[i];

    return mahal_sq <= threshold;
}
