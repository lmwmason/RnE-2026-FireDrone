#ifndef MATH_MATRIX_H
#define MATH_MATRIX_H

#include <stdint.h>

#define MAT_MAX_DIM 21U

void mat_add(const float *A, const float *B, float *C, uint8_t r, uint8_t c);
void mat_sub(const float *A, const float *B, float *C, uint8_t r, uint8_t c);
void mat_scale(const float *A, float s, float *C, uint8_t r, uint8_t c);
void mat_copy(const float *src, float *dst, uint8_t r, uint8_t c);
void mat_zero(float *A, uint8_t r, uint8_t c);
void mat_identity(float *A, uint8_t n);
void mat_trans(const float *A, float *At, uint8_t r, uint8_t c);
void mat_sym(float *A, uint8_t n);
void mat_mul(const float *A, const float *B, float *C, uint8_t n, uint8_t m, uint8_t p);
void mat_mul_aT(const float *A, const float *B, float *C, uint8_t m, uint8_t n, uint8_t p);
void mat_mul_bT(const float *A, const float *B, float *C, uint8_t n, uint8_t m, uint8_t p);
int  mat_chol(const float *A, float *L, uint8_t n);
int  mat_inv(const float *A, float *Ainv, uint8_t n);

#endif
