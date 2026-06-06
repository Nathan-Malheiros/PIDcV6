#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * Persistent storage for runtime-calibrated values (NVS, namespace "bb_cal").
 * Call cal_store_init() once before any load/save operation.
 */

typedef struct {
    float kp, ki, kd;
} cal_pid_t;

typedef struct {
    int32_t x_raw_min, x_raw_max;
    int32_t y_raw_min, y_raw_max;
    int8_t  flip_x, flip_y, swap_xy;
} cal_touch_t;

/* Limites de curso dos motores em PASSOS, definidos pela calibração "STEPPER".
 * step_min = ponto baixo (bate na mesa), step_max = ponto alto (limite do braço).
 * O startup opera no meio do curso. */
typedef struct {
    int32_t step_min, step_max;
} cal_steplim_t;

/* Baselines "sem bola" (comando ZERO): lista de pontos presos do painel a
 * ignorar. Salvo no NVS para valer após reiniciar. */
#define CAL_BASE_MAX  8
typedef struct {
    int8_t count;
    float  x[CAL_BASE_MAX], y[CAL_BASE_MAX];
} cal_baseline_t;

/* Viés de nível (comando TRIM): inclinação CONSTANTE (rad) aplicada como
 * feedforward para compensar a base/estrutura torta. Aprendido do integral em
 * regime e salvo no NVS — a plataforma já nasce nivelada após reiniciar. */
typedef struct {
    float nx, ny;
} cal_trim_t;

/* Initialize NVS flash — erases partition only if version mismatch or full. */
void cal_store_init(void);

/* Returns true if a valid PID blob was found in NVS. */
bool cal_store_load_pid(cal_pid_t *out);
void cal_store_save_pid(const cal_pid_t *in);

/* Returns true if a valid touch calibration blob was found in NVS. */
bool cal_store_load_touch(cal_touch_t *out);
void cal_store_save_touch(const cal_touch_t *in);

/* Returns true if motor travel limits were found in NVS. */
bool cal_store_load_steplim(cal_steplim_t *out);
void cal_store_save_steplim(const cal_steplim_t *in);

/* Returns true if a touch baseline blob was found in NVS. */
bool cal_store_load_baseline(cal_baseline_t *out);
void cal_store_save_baseline(const cal_baseline_t *in);

/* Returns true if a level-trim blob was found in NVS. */
bool cal_store_load_trim(cal_trim_t *out);
void cal_store_save_trim(const cal_trim_t *in);
