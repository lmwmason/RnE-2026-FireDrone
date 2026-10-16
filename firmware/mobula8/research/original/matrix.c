#include "matrix.h"
#include <math.h>
#include <string.h>

void mat_add(const float *A, const float *B, float *C, uint8_t r, uint8_t c)
{
    uint16_t n = (uint16_t)r * c;
    for (uint16_t i = 0; i < n; i++) C[i] = A[i] + B[i];
}

void mat_sub(const float *A, const float *B, float *C, uint8_t r, uint8_t c)
{
    uint16_t n = (uint16_t)r * c;
    for (uint16_t i = 0; i < n; i++) C[i] = A[i] - B[i];
}

void mat_scale(const float *A, float s, float *C, uint8_t r, uint8_t c)
{
    uint16_t n = (uint16_t)r * c;
    for (uint16_t i = 0; i < n; i++) C[i] = A[i] * s;
}

void mat_copy(const float *src, float *dst, uint8_t r, uint8_t c)
{
    memcpy(dst, src, (uint16_t)r * c * sizeof(float));
}

void mat_zero(float *A, uint8_t r, uint8_t c)
{
    memset(A, 0, (uint16_t)r * c * sizeof(float));
}

void mat_identity(float *A, uint8_t n)
{
    mat_zero(A, n, n);
    for (uint8_t i = 0; i < n; i++) A[(uint16_t)i * n + i] = 1.0f;
}

void mat_trans(const float *A, float *At, uint8_t r, uint8_t c)
{
    for (uint8_t i = 0; i < r; i++)
        for (uint8_t j = 0; j < c; j++)
            At[(uint16_t)j * r + i] = A[(uint16_t)i * c + j];
}

void mat_sym(float *A, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1; j < n; j++) {
            float v = 0.5f * (A[(uint16_t)i * n + j] + A[(uint16_t)j * n + i]);
            A[(uint16_t)i * n + j] = v;
            A[(uint16_t)j * n + i] = v;
        }
    }
}

void mat_mul(const float *A, const float *B, float *C, uint8_t n, uint8_t m, uint8_t p)
{
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t k = 0; k < p; k++) {
            float s = 0.0f;
            for (uint8_t j = 0; j < m; j++)
                s += A[(uint16_t)i * m + j] * B[(uint16_t)j * p + k];
            C[(uint16_t)i * p + k] = s;
        }
    }
}

void mat_mul_aT(const float *A, const float *B, float *C, uint8_t m, uint8_t n, uint8_t p)
{
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t k = 0; k < p; k++) {
            float s = 0.0f;
            for (uint8_t j = 0; j < m; j++)
                s += A[(uint16_t)j * n + i] * B[(uint16_t)j * p + k];
            C[(uint16_t)i * p + k] = s;
        }
    }
}

void mat_mul_bT(const float *A, const float *B, float *C, uint8_t n, uint8_t m, uint8_t p)
{
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t k = 0; k < p; k++) {
            float s = 0.0f;
            for (uint8_t j = 0; j < m; j++)
                s += A[(uint16_t)i * m + j] * B[(uint16_t)k * m + j];
            C[(uint16_t)i * p + k] = s;
        }
    }
}

int mat_chol(const float *A, float *L, uint8_t n)
{
    mat_zero(L, n, n);
    for (uint8_t j = 0; j < n; j++) {
        float s = A[(uint16_t)j * n + j];
        for (uint8_t k = 0; k < j; k++) {
            float lkj = L[(uint16_t)j * n + k];
            s -= lkj * lkj;
        }
        if (s <= 0.0f) return -1;
        L[(uint16_t)j * n + j] = sqrtf(s);
        float inv_ljj = 1.0f / L[(uint16_t)j * n + j];
        for (uint8_t i = j + 1; i < n; i++) {
            float t = A[(uint16_t)i * n + j];
            for (uint8_t k = 0; k < j; k++)
                t -= L[(uint16_t)i * n + k] * L[(uint16_t)j * n + k];
            L[(uint16_t)i * n + j] = t * inv_ljj;
        }
    }
    return 0;
}

int mat_inv(const float *A, float *Ainv, uint8_t n)
{
    float aug[MAT_MAX_DIM * 2 * MAT_MAX_DIM];
    uint8_t n2 = (uint8_t)(n * 2u);

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            aug[(uint16_t)i * n2 + j]     = A[(uint16_t)i * n + j];
            aug[(uint16_t)i * n2 + n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (uint8_t col = 0; col < n; col++) {
        uint8_t pivot = col;
        float max_v = fabsf(aug[(uint16_t)col * n2 + col]);
        for (uint8_t row = (uint8_t)(col + 1u); row < n; row++) {
            float v = fabsf(aug[(uint16_t)row * n2 + col]);
            if (v > max_v) { max_v = v; pivot = row; }
        }
        if (max_v < 1e-10f) return -1;

        if (pivot != col) {
            for (uint8_t j = 0; j < n2; j++) {
                float tmp = aug[(uint16_t)col * n2 + j];
                aug[(uint16_t)col   * n2 + j] = aug[(uint16_t)pivot * n2 + j];
                aug[(uint16_t)pivot * n2 + j] = tmp;
            }
        }

        float inv_p = 1.0f / aug[(uint16_t)col * n2 + col];
        for (uint8_t j = 0; j < n2; j++) aug[(uint16_t)col * n2 + j] *= inv_p;

        for (uint8_t row = 0; row < n; row++) {
            if (row == col) continue;
            float f = aug[(uint16_t)row * n2 + col];
            for (uint8_t j = 0; j < n2; j++)
                aug[(uint16_t)row * n2 + j] -= f * aug[(uint16_t)col * n2 + j];
        }
    }

    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            Ainv[(uint16_t)i * n + j] = aug[(uint16_t)i * n2 + n + j];

    return 0;
}
