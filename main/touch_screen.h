#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ── Physical dimensions (VSDISPLAY VS084TP-A1, 8.4-inch) ───────────────── */
#define TOUCH_WIDTH_MM    187.0f
#define TOUCH_HEIGHT_MM   141.0f

/* ── GPIO assignments (verified physically, FPC connector) ──────────────── */
/*    FPC pin 1 = Y+  →  GPIO 4   ADC1_CH3   (sense for X measurement)      */
/*    FPC pin 2 = X-  →  GPIO 5   ADC1_CH4   (sense for Y measurement)      */
/*    FPC pin 3 = Y-  →  GPIO 8   ADC1_CH7   (driven during Y measurement)  */
/*    FPC pin 4 = X+  →  GPIO 9   output only — ADC inoperante nesta placa  */
#define TOUCH_PIN_XP   9
#define TOUCH_PIN_XM   5
#define TOUCH_PIN_YP   4
#define TOUCH_PIN_YM   8

/* ── ADC calibration boundaries (12-bit raw, 0–4095) ────────────────────── */
/*    Updated via the calibration wizard in tools/calibrate.py.                */
/*    These shift slightly between power cycles — recalibrate when needed.   */
#define TOUCH_X_RAW_MIN  665
#define TOUCH_X_RAW_MAX  2981
#define TOUCH_Y_RAW_MIN  554
#define TOUCH_Y_RAW_MAX  3273

/* ── Tuning parameters ───────────────────────────────────────────────────── */

/* ADC samples averaged per axis per read cycle. */
#define TOUCH_SAMPLES        8

/*
 * Axis orientation — keep in sync with tools/config.py.
 * Applied to the normalised position before mm conversion so the output
 * frame matches the physical table. The three flags together cover all 8
 * orientations (4 rotations × mirror).
 *
 * Defaults abaixo assumem o painel montado como a VISTA FRONTAL do datasheet:
 * paisagem 187 x 141 mm, cabo flat (FPC) saindo pela DIREITA, eletrodos
 * X+ em cima / X- embaixo / Y+ à esquerda / Y- à direita.
 *
 * A leitura crua deste painel sai com os eixos TROCADOS:
 *   x_raw = eixo VERTICAL   (X+ topo .. X- base)
 *   y_raw = eixo HORIZONTAL (Y+ esq  .. Y- dir)
 * por isso, para a saída ficar x_mm=horizontal e y_mm=vertical:
 *   TOUCH_SWAP_XY = 1  -> x_mm vira o horizontal, y_mm o vertical
 *   TOUCH_FLIP_X  = 1  -> x_mm = 0 na ESQUERDA, 187 na DIREITA (lado do cabo)
 *   TOUCH_FLIP_Y  = 1  -> y_mm = 0 em CIMA,     141 em BAIXO
 *
 * Se montar com o cabo à ESQUERDA (painel girado 180°):
 *   TOUCH_SWAP_XY = 1, TOUCH_FLIP_X = 0, TOUCH_FLIP_Y = 0
 *
 * Em qualquer caso o assistente CAL (CAL TL/TR/BL/BR -> APPLY) detecta
 * swap/flip sozinho a partir dos 4 cantos, e o valor salvo no NVS tem
 * PRIORIDADE sobre estes defaults — recalibre (ou CAL RESET) após remontar.
 */
#define TOUCH_SWAP_XY  1
#define TOUCH_FLIP_X   1
#define TOUCH_FLIP_Y   1

/*
 * IIR low-pass filter weight applied to the converted mm value.
 * Range: 0.0 (frozen) – 1.0 (no filter). MAIOR = menos atraso, mais ruído.
 * Subido de 0.25 -> 0.45 para reduzir o lag de posição (~60 ms -> ~25 ms),
 * melhorando o tempo de resposta. A detecção digital + média de 8 amostras de
 * ADC já seguram bem o ruído.
 */
#define TOUCH_IIR_ALPHA      0.45f

/*
 * ADC threshold for touch detection. Um contato real carrega o pino de sense
 * para >1700; o acoplamento elétrico (motores girando) dá valores médios. Subido
 * de 300 -> 800 para rejeitar esse acoplamento. Toque só vale se AMBOS os eixos
 * lerem acima disto E a detecção digital (pull-up) confirmar.
 */
#define TOUCH_DETECT_THRESHOLD  800

/*
 * Detecção digital robusta (pull-up): amostra TOUCH_DETECT_SAMPLES vezes e exige
 * pelo menos TOUCH_DETECT_NEED em nível baixo. Rejeita picos de ruído (1 amostra
 * suja não dispara o controle).
 */
#define TOUCH_DETECT_SAMPLES   6
#define TOUCH_DETECT_NEED      5

/*
 * Microsecond timings for the resistive scan.
 *   DISCHARGE: time the sense node is pulled to 0 before floating to read.
 *   SETTLE:    time for the node to charge through the contact before sampling.
 * Resistive panels settle in well under 100 us; these are conservative.
 */
#define TOUCH_DISCHARGE_US    70
#define TOUCH_SETTLE_US       70

/* ── Data type ───────────────────────────────────────────────────────────── */

typedef struct {
    float   x_mm;    /* Filtered X position  [0 .. TOUCH_WIDTH_MM]  */
    float   y_mm;    /* Filtered Y position  [0 .. TOUCH_HEIGHT_MM] */
    int32_t x_raw;   /* Averaged raw ADC for X axis                 */
    int32_t y_raw;   /* Averaged raw ADC for Y axis                 */
    bool    valid;   /* true = object detected on surface           */
} touch_pos_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * Initialise GPIO and ADC peripheral.
 * Call once before any touch_read() call.
 */
void touch_init(void);

/**
 * Perform one full X+Y measurement cycle.
 *
 * Switching sequence:
 *   1. GPIO touch-detection check (no ADC needed).
 *   2. Drive X axis, ADC-sense on Y+ (GPIO 4) → X coordinate.
 *   3. Drive Y axis, ADC-sense on X- (GPIO 5) → Y coordinate.
 *   4. Apply IIR filter and convert raw → mm.
 *
 * @param[out] pos  Populated on every call; pos->valid indicates contact.
 * @return          Same value as pos->valid.
 */
bool touch_read(touch_pos_t *pos);

/*
 * Runtime calibration override — replaces the compile-time #define defaults.
 * Can be called at any time (takes effect on the next touch_read()).
 * Typically loaded from NVS via cal_store or computed by the CAL APPLY command.
 *
 * Orientation flags follow the same convention as the compile-time macros:
 *   swap_xy : swap X <-> Y axes (90° panel rotation)
 *   flip_x  : mirror X axis
 *   flip_y  : mirror Y axis
 */
void touch_set_cal(int32_t x_raw_min, int32_t x_raw_max,
                   int32_t y_raw_min, int32_t y_raw_max,
                   int flip_x, int flip_y, int swap_xy);

void touch_get_cal(int32_t *x_raw_min, int32_t *x_raw_max,
                   int32_t *y_raw_min, int32_t *y_raw_max,
                   int *flip_x, int *flip_y, int *swap_xy);

/* Baselines "sem bola" (comando ZERO): pontos presos do painel a ignorar.
 * touch_add_baseline() adiciona o ponto atual à lista (retorna o total novo,
 * ou <0 se sem leitura/cheio). touch_clear_baseline() limpa.
 * touch_get_baselines/set_baselines: para persistir no NVS. */
int  touch_add_baseline(void);
void touch_clear_baseline(void);
int  touch_get_baselines(float *xs, float *ys, int max);
void touch_set_baselines(int n, const float *xs, const float *ys);
