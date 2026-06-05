#include "control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "touch_screen.h"
#include "kinematics.h"
#include "pid.h"
#include "steppers.h"
#include "cal_store.h"

static const char *TAG = "control";

/* ── Machine geometry (mm) ──────────────────────────────────────────────────
 * PLACEHOLDERS — MEDIR no rig real e substituir.
 */
#define GEO_D   50.8
#define GEO_E   79.4
#define GEO_F   44.45
#define GEO_G   93.2
#define GEO_HZ  108.0

/* ── Stepper scaling ──────────────────────────────────────────────────────── */
#define MICROSTEPS       16
#define STEPS_PER_REV    (200 * MICROSTEPS)
#define ANG_TO_STEP      (STEPS_PER_REV / 360.0)

/* ── Control loop ─────────────────────────────────────────────────────────── */
#define LOOP_HZ          100      /* 100 Hz: metade do atraso de amostragem de 50 Hz */
#define LOOP_DT          (1.0f / LOOP_HZ)
#define LOOP_PERIOD_MS   (1000 / LOOP_HZ)

/* Debounce de presença da bola: só controla com a bola DETECTADA de fato.
 * Sem bola, a mesa fica em IDLE (nivelada). Em ciclos do laço (agora 100 Hz). */
#define PRESENCE_ON   4    /* leituras COM toque para ATIVAR  (~40 ms) */
#define PRESENCE_OFF  14   /* leituras SEM toque para ir IDLE (~140 ms) */

#define SETPOINT_X_MM    (TOUCH_WIDTH_MM  / 2.0f)
#define SETPOINT_Y_MM    (TOUCH_HEIGHT_MM / 2.0f)

#define TILT_LIMIT       0.25f

/* ── Sinal do controle (realimentação NEGATIVA) ──────────────────────────────
 * O PID usa err = meas - setpoint, logo a saída ox = Kp*(meas-setpoint) tem o
 * MESMO sentido do erro. Para a mesa inclinar de modo a trazer a bola de volta
 * ao centro (realimentação negativa), a inclinação aplicada precisa ser o
 * NEGATIVO da saída do PID:  nx = -ox.
 *
 * Verificação: bola à direita (err>0) -> queremos a bola acelerando p/ a
 * esquerda. Com nx = -Kp*err < 0, o gradiente do tampo dz/dX > 0 e a bola
 * acelera no sentido -x (de volta ao centro). É a MESMA convenção da simulação
 * PIDSimba (alpha = -pid.update(...)), que converge com esta mesma cinemática.
 *
 * Se, no SEU hardware, um eixo ainda empurrar a bola para longe (fiação do
 * motor / orientação da tela invertem o sentido físico), inverta aquele eixo
 * ao vivo pela serial:  SX +1  ou  SY +1  — sem recompilar.
 */
#define TILT_SIGN_X      (-1.0f)
#define TILT_SIGN_Y      (-1.0f)

/* ── PID default gains (mm domain) ───────────────────────────────────────────
 * Ajuste via serial em tempo real; os valores salvos no NVS têm prioridade.
 */
#define KP   8.0e-4f
#define KI   2.0e-5f
#define KD   1.2e-2f

/* ── Stepper speed/accel limits ──────────────────────────────────────────────
 * Valores SUAVES (teste). Pressupõem o DRV8825 em 1/16 (MICROSTEPS=16). Se o
 * movimento estiver violento, o culpado quase sempre é o microstep do driver
 * (M0/M1/M2) NÃO estar em 1/16 — verifique o hardware antes de subir estes
 * números. Depois de tudo liso, dá para aumentar para resposta mais rápida.
 */
#define SPEED_BALANCE    1800.0f   /* mais rapido p/ a mesa acompanhar a bola */
#define ACCEL_BALANCE   16000.0f   /* resposta mais viva (suave pois micro=1/16) */
#define SPEED_HOME        400.0f   /* homing continua devagar (seguro) */
#define ACCEL_HOME       1500.0f

/* ── Homing: subida inicial ───────────────────────────────────────────────
 * Antes de nivelar, a plataforma sobe HOMING_LIFT_MM acima da altura de
 * trabalho (GEO_HZ). Isso garante que o primeiro movimento dos braços seja
 * sempre para CIMA, evitando colisão com a base de suporte dos motores.
 * Mantido PEQUENO: só o suficiente para o primeiro passo ser para cima.
 */
#define HOMING_LIFT_MM   5.0

/* ── Layout físico dos motores e orientação da mesa ──────────────────────
 *
 *  Painel resistivo 4 fios (datasheet) — VISTA FRONTAL, paisagem 187 x 141 mm,
 *  cabo flat (FPC) saindo pela DIREITA. Eletrodos: X+ em cima, X- embaixo,
 *  Y+ à esquerda, Y- à direita  (FPC: 1=Y+ 2=X- 3=Y- 4=X+).
 *
 *              CIMA  (eletrodo X+)  ->  y_mm = 0
 *                 Motor A (STEP=15 DIR=16)
 *                       │
 *  (Y+)     ┌───────────┴───────────┐     (Y-)
 * x_mm=0    │     MESA RESISTIVA    │ ──► cabo FPC (DIREITA)
 *           │   ●  ←  bola  →  ●    │    x_mm=187
 *           └──────┬─────────┬──────┘
 *      Motor C     │         │    Motor B
 *    (STEP=40   BAIXO-ESQ  BAIXO-DIR  (STEP=17
 *     DIR=41)     (240°)     (120°)    DIR=18)
 *              BAIXO  (eletrodo X-)  ->  y_mm = 141
 *
 *  Coordenadas da tela após calibração:
 *    x_mm = 0 → ESQUERDA   x_mm = 187 → DIREITA (lado do cabo FPC)
 *    y_mm = 0 → CIMA       y_mm = 141 → BAIXO
 *
 *  Setpoint de equilíbrio: x=93.5 mm (centro),  y=70.5 mm (centro)
 */


/* ── Estado compartilhado ─────────────────────────────────────────────────── */

static machine_t s_machine;
static pid_t     s_pid_x, s_pid_y;
static double    s_ang_orig;
static float     s_sign_x = TILT_SIGN_X;
static float     s_sign_y = TILT_SIGN_Y;

/* Último toque lido pelo loop principal; usado pelo cmd_task para capturar
 * cantos. Campos 32-bit são atômicos em ARM — sem mutex para leitura simples. */
static volatile int32_t s_snap_x_raw   = 0;
static volatile int32_t s_snap_y_raw   = 0;
static volatile bool    s_snap_touched = false;

/* Telemetria habilitada apenas entre START/STOP */
static volatile bool    s_telem_enabled = false;

/* ── Calibração de cantos da mesa ─────────────────────────────────────────
 *
 * Orientação (cabo flat da tela resistiva à DIREITA, olhando de frente):
 *
 *     TL ──────────── TR   ← cabo FPC aqui (direita)
 *     │                │
 *     │     MESA       │
 *     │                │
 *     BL ──────────── BR
 *
 * Convenção de saída após calibração:
 *   x_mm = 0 na borda ESQUERDA, TOUCH_WIDTH_MM na borda DIREITA
 *   y_mm = 0 na borda SUPERIOR,  TOUCH_HEIGHT_MM na borda INFERIOR
 */
#define COR_TL 0
#define COR_TR 1
#define COR_BL 2
#define COR_BR 3

typedef struct { int32_t x, y; bool valid; } corner_t;
static corner_t s_corners[4];

/* ── Calibração guiada pelo terminal serial (humano) ──────────────────────
 * Máquina de estados conduzida pelos comandos OK / SKIP / CANCELAR. Enquanto
 * ativa (s_calibrating), o laço mantém a mesa NIVELADA (não balanceia), para a
 * bola ficar onde você a coloca, e emite um stream LIVE enxuto a ~2,5 Hz.
 */
typedef enum {
    CAL_IDLE, CAL_WAIT_TL, CAL_WAIT_TR, CAL_WAIT_BR, CAL_WAIT_BL,
    CAL_WAIT_CENTER, CAL_CONFIRM
} cal_state_t;
static volatile cal_state_t s_cal_state    = CAL_IDLE;
static volatile bool        s_calibrating  = false;
static corner_t   s_center;                       /* ponto central (verificação) */
static int32_t    s_bak_xmn, s_bak_xmx, s_bak_ymn, s_bak_ymx;  /* backup p/ cancelar */
static int        s_bak_fx, s_bak_fy, s_bak_sw;
static TickType_t s_cal_live_last = 0;

/* ── Modo de teste: movimento em ELIPSE (varre os 3 motores, sem PID) ──────
 * Inclina a mesa descrevendo uma elipse suave: nx = Ax·cos(φ), ny = Ay·sin(φ).
 * Cada motor sobe e desce em sequência — ótimo para testar mecânica/drivers.
 * Liga/desliga digitando "ELIPSE" (ou "PARAR" para sair). Não toca no PID.
 */
static volatile bool s_ellipse       = false;
static double        s_ellipse_phase = 0.0;
#define ELLIPSE_AMP_X   0.15      /* amplitude da inclinação em X (< TILT_LIMIT) */
#define ELLIPSE_AMP_Y   0.10      /* amplitude em Y (≠ X → elipse) */
#define ELLIPSE_HZ      0.17      /* ~6 s por volta (movimento lento e suave) */
#define ELLIPSE_TWO_PI  6.283185307179586

/* ── Calibração de curso dos motores (modo guiado "STEPPER") ──────────────
 * Você sobe/desce a plataforma de 1 em 1 mm e marca MINIMO (bate na mesa) e
 * MAXIMO (limite do braço). O startup passa a subir->descer->parar no MEIO
 * desse curso, sem forçar. s_work_hz = altura de trabalho efetiva (mm).
 */
static volatile bool s_stepper      = false;
static double        s_jog_hz       = GEO_HZ;   /* altura sendo ajustada (mm) */
static double        s_work_hz      = GEO_HZ;   /* altura de trabalho efetiva  */
static double        s_hz_min       = 0.0;
static double        s_hz_max       = 0.0;
static bool          s_hz_min_set   = false;
static bool          s_hz_max_set   = false;
#define STEPPER_JOG_MM   1.0       /* incremento por SOBE/DESCE */
#define HZ_ABS_MIN      60.0       /* trava de seguranca (evita NaN/curso absurdo) */
#define HZ_ABS_MAX     150.0

/* ── Auto-tune de PID por relé (método de Åström–Hägglund) ────────────────
 * "PIDAUTO": a mesa aplica um relé (liga/desliga a inclinação) que faz a bola
 * oscilar em torno do centro. Mede-se o período (Tu) e a amplitude (a) dessa
 * oscilação; daí o ganho crítico Ku = 4h/(π·a). Ziegler–Nichols converte
 * (Ku, Tu) em Kp/Ki/Kd. Sem chutar valores.
 */
static volatile bool s_pidauto = false;
static int        s_pa_relay_x, s_pa_relay_y;   /* estado do relé por eixo */
static int        s_pa_prevsign;                /* sinal anterior do erro X */
static float      s_pa_peak;                     /* pico |erro X| no meio-ciclo */
static TickType_t s_pa_last_cross, s_pa_start, s_pa_seen;
static float      s_pa_sum_half, s_pa_sum_amp;
static int        s_pa_meas, s_pa_halfcount;
#define PA_H          0.05f      /* amplitude do relé (rad) — oscilação suave */
#define PA_HYST       4.0f       /* histerese (mm) — robustez a ruído */
#define PA_CYCLES     8          /* meio-ciclos medidos */
#define PA_WARMUP     2          /* descarta os primeiros (transitório) */
#define PA_SAFE_MM    60.0f      /* aborta se a bola passar disto (diverge/borda) */
#define PA_TIMEOUT_S  45.0f

/* ── Impressão de estado ──────────────────────────────────────────────────── */

static void cmd_print_gains(void)
{
    printf("GAINS,%.6g,%.6g,%.6g\n",
           (double)s_pid_x.kp, (double)s_pid_x.ki, (double)s_pid_x.kd);
}

static void cmd_print_signs(void)
{
    printf("SIGNS,%+.0f,%+.0f\n", (double)s_sign_x, (double)s_sign_y);
}

static void cmd_print_cal(void)
{
    int32_t xmin, xmax, ymin, ymax;
    int fx, fy, sw;
    touch_get_cal(&xmin, &xmax, &ymin, &ymax, &fx, &fy, &sw);
    printf("CAL,%ld,%ld,%ld,%ld,%d,%d,%d\n",
           (long)xmin, (long)xmax, (long)ymin, (long)ymax, fx, fy, sw);
}

/* ── Comandos de calibração ───────────────────────────────────────────────── */

static void cmd_set_all_gains(float kp, float ki, float kd)
{
    s_pid_x.kp = kp; s_pid_x.ki = ki; s_pid_x.kd = kd;
    s_pid_y.kp = kp; s_pid_y.ki = ki; s_pid_y.kd = kd;
}

static void cmd_capture_corner(int idx)
{
    if (!s_snap_touched) { printf("ERR,no_touch\n"); return; }
    s_corners[idx].x     = s_snap_x_raw;
    s_corners[idx].y     = s_snap_y_raw;
    s_corners[idx].valid = true;
    const char *names[] = { "TL", "TR", "BL", "BR" };
    printf("CORNER,%s,%ld,%ld\n", names[idx],
           (long)s_corners[idx].x, (long)s_corners[idx].y);
}

static void cmd_cal_apply(void)
{
    if (!s_corners[COR_TL].valid || !s_corners[COR_TR].valid ||
        !s_corners[COR_BL].valid || !s_corners[COR_BR].valid) {
        printf("ERR,missing_corners\n");
        return;
    }

    /* Média de cada lado/aresta */
    int32_t left_x  = (s_corners[COR_TL].x + s_corners[COR_BL].x) / 2;
    int32_t right_x = (s_corners[COR_TR].x + s_corners[COR_BR].x) / 2;
    int32_t top_x   = (s_corners[COR_TL].x + s_corners[COR_TR].x) / 2;
    int32_t bot_x   = (s_corners[COR_BL].x + s_corners[COR_BR].x) / 2;
    int32_t left_y  = (s_corners[COR_TL].y + s_corners[COR_BL].y) / 2;
    int32_t right_y = (s_corners[COR_TR].y + s_corners[COR_BR].y) / 2;
    int32_t top_y   = (s_corners[COR_TL].y + s_corners[COR_TR].y) / 2;
    int32_t bot_y   = (s_corners[COR_BL].y + s_corners[COR_BR].y) / 2;

    /* Auto-detecção de swap_xy: se x_raw varia mais verticalmente do que
     * horizontalmente, o eixo elétrico X está mapeado no eixo físico Y.    */
    int32_t x_var_horiz = abs(right_x - left_x);
    int32_t x_var_vert  = abs(bot_x   - top_x);
    int swap_xy = (x_var_vert > x_var_horiz) ? 1 : 0;

    int32_t x_raw_min, x_raw_max, y_raw_min, y_raw_max;
    int flip_x, flip_y;

    if (!swap_xy) {
        /* x_raw → horizontal, y_raw → vertical */
        x_raw_min = left_x  < right_x ? left_x  : right_x;
        x_raw_max = left_x  > right_x ? left_x  : right_x;
        y_raw_min = top_y   < bot_y   ? top_y   : bot_y;
        y_raw_max = top_y   > bot_y   ? top_y   : bot_y;
        /* flip: se o lado esquerdo tem raw maior, precisamos inverter
         * para que x_mm=0 fique na esquerda.                           */
        flip_x = (left_x > right_x) ? 1 : 0;
        /* flip: se a borda superior tem raw maior, invertemos
         * para que y_mm=0 fique no topo.                               */
        flip_y = (top_y  > bot_y  ) ? 1 : 0;
    } else {
        /* x_raw → vertical, y_raw → horizontal (após swap: new_x=old_y, new_y=old_x) */
        x_raw_min = top_x  < bot_x   ? top_x  : bot_x;
        x_raw_max = top_x  > bot_x   ? top_x  : bot_x;
        y_raw_min = left_y < right_y ? left_y : right_y;
        y_raw_max = left_y > right_y ? left_y : right_y;
        flip_x = (left_y > right_y) ? 1 : 0;
        flip_y = (top_x  > bot_x  ) ? 1 : 0;
    }

    touch_set_cal(x_raw_min, x_raw_max, y_raw_min, y_raw_max, flip_x, flip_y, swap_xy);
    cmd_print_cal();
}

static void cmd_cal_save(void)
{
    int32_t xmin, xmax, ymin, ymax;
    int fx, fy, sw;
    touch_get_cal(&xmin, &xmax, &ymin, &ymax, &fx, &fy, &sw);
    cal_touch_t tc = { xmin, xmax, ymin, ymax, (int8_t)fx, (int8_t)fy, (int8_t)sw };
    cal_store_save_touch(&tc);
    printf("SAVED,touch\n");
    cmd_print_cal();
}

static void cmd_cal_reset(void)
{
    touch_set_cal(TOUCH_X_RAW_MIN, TOUCH_X_RAW_MAX,
                  TOUCH_Y_RAW_MIN, TOUCH_Y_RAW_MAX,
                  TOUCH_FLIP_X, TOUCH_FLIP_Y, TOUCH_SWAP_XY);
    for (int i = 0; i < 4; i++) s_corners[i].valid = false;
    cmd_print_cal();
}

static void cmd_handle_cal(const char *sub)
{
    if      (strcmp(sub, "TL")    == 0) { cmd_capture_corner(COR_TL); }
    else if (strcmp(sub, "TR")    == 0) { cmd_capture_corner(COR_TR); }
    else if (strcmp(sub, "BL")    == 0) { cmd_capture_corner(COR_BL); }
    else if (strcmp(sub, "BR")    == 0) { cmd_capture_corner(COR_BR); }
    else if (strcmp(sub, "APPLY") == 0) { cmd_cal_apply(); }
    else if (strcmp(sub, "SAVE")  == 0) { cmd_cal_save(); }
    else if (strcmp(sub, "SHOW")  == 0) { cmd_print_cal(); }
    else if (strcmp(sub, "RESET") == 0) { cmd_cal_reset(); }
    else { printf("ERR,unknown_cal_cmd\n"); }
}

/* ── Menu PID (humano) ────────────────────────────────────────────────────── */

static void cmd_pid_menu(void)
{
    printf("----------------------------------------------\n");
    printf(" PID atual:  Kp=%.6g   Ki=%.6g   Kd=%.6g\n",
           (double)s_pid_x.kp, (double)s_pid_x.ki, (double)s_pid_x.kd);
    printf(" Para mudar:  PID <kp> <ki> <kd>\n");
    printf("              KP <v>   |   KI <v>   |   KD <v>\n");
    printf(" Gravar NVS:  PID SAVE\n");
    printf("----------------------------------------------\n");
}

/* ── Calibração guiada: prompts e máquina de estados ──────────────────────── */

static void cal_print_prompt(void)
{
    switch (s_cal_state) {
    case CAL_WAIT_TL:
        printf("CAL> [1/5] Bola no canto SUPERIOR-ESQUERDO  -> digite OK\n"); break;
    case CAL_WAIT_TR:
        printf("CAL> [2/5] Bola no canto SUPERIOR-DIREITO (lado do cabo) -> OK\n"); break;
    case CAL_WAIT_BR:
        printf("CAL> [3/5] Bola no canto INFERIOR-DIREITO (lado do cabo) -> OK\n"); break;
    case CAL_WAIT_BL:
        printf("CAL> [4/5] Bola no canto INFERIOR-ESQUERDO -> OK\n"); break;
    case CAL_WAIT_CENTER:
        printf("CAL> [5/5] Bola no CENTRO -> OK   (ou SKIP para pular)\n"); break;
    case CAL_CONFIRM:
        printf("CAL> SALVAR (grava no NVS)  ou  CANCELAR (descarta)\n"); break;
    default: break;
    }
}

static bool cal_capture(corner_t *c, const char *name)
{
    if (!s_snap_touched) {
        printf("CAL> sem toque — coloque a bola no lugar e digite OK\n");
        return false;
    }
    c->x = s_snap_x_raw; c->y = s_snap_y_raw; c->valid = true;
    printf("CAL> %s capturado: raw=(%ld,%ld)\n", name, (long)c->x, (long)c->y);
    return true;
}

static void cal_advance(void)
{
    switch (s_cal_state) {
    case CAL_WAIT_TL:     s_cal_state = CAL_WAIT_TR;     break;
    case CAL_WAIT_TR:     s_cal_state = CAL_WAIT_BR;     break;
    case CAL_WAIT_BR:     s_cal_state = CAL_WAIT_BL;     break;
    case CAL_WAIT_BL:     s_cal_state = CAL_WAIT_CENTER; break;
    case CAL_WAIT_CENTER:
        /* Calcula a calibração a partir dos 4 cantos (já validados) */
        cmd_cal_apply();   /* aplica ao vivo; imprime CAL,... */
        s_cal_state = CAL_CONFIRM;
        printf("CAL> Calibracao aplicada. Confira o CENTRO no stream LIVE "
               "(ideal ~%.0f,%.0f mm).\n",
               (double)SETPOINT_X_MM, (double)SETPOINT_Y_MM);
        break;
    default: break;
    }
    cal_print_prompt();
}

static void cal_start(void)
{
    /* backup da calibração atual, para restaurar se o usuário CANCELAR */
    touch_get_cal(&s_bak_xmn, &s_bak_xmx, &s_bak_ymn, &s_bak_ymx,
                  &s_bak_fx, &s_bak_fy, &s_bak_sw);
    for (int i = 0; i < 4; i++) s_corners[i].valid = false;
    s_center.valid = false;
    s_ellipse     = false;        /* nao roda elipse durante a calibracao */
    s_stepper     = false;
    s_pidauto     = false;
    s_cal_state   = CAL_WAIT_TL;
    s_calibrating = true;
    pid_reset(&s_pid_x);   /* a mesa para de balancear durante a calibração */
    pid_reset(&s_pid_y);
    printf("CAL> Modo CALIBRACAO. A mesa fica NIVELADA (nao balanceia).\n");
    printf("CAL> Orientacao: cabo FPC a DIREITA.  Comandos: OK | SKIP | CANCELAR\n");
    cal_print_prompt();
}

static void cal_finish(bool save)
{
    if (save) {
        cmd_cal_save();
        printf("CAL> calibracao SALVA no NVS. Equilibrio retomado.\n");
    } else {
        touch_set_cal(s_bak_xmn, s_bak_xmx, s_bak_ymn, s_bak_ymx,
                      s_bak_fx, s_bak_fy, s_bak_sw);
        printf("CAL> CANCELADO. Calibracao anterior restaurada.\n");
    }
    s_cal_state   = CAL_IDLE;
    s_calibrating = false;
    pid_reset(&s_pid_x);
    pid_reset(&s_pid_y);
}

/* Trata uma linha enquanto estamos no modo calibração guiada. */
static void cal_step(const char *s)
{
    if (strcmp(s, "CANCELAR") == 0 || strcmp(s, "CANCEL") == 0 ||
        strcmp(s, "ABORT") == 0) {
        cal_finish(false);
        return;
    }

    if (s_cal_state == CAL_CONFIRM) {
        if (strcmp(s, "SALVAR") == 0 || strcmp(s, "SAVE") == 0) {
            cal_finish(true);
        } else {
            printf("CAL> digite SALVAR para gravar ou CANCELAR para descartar\n");
        }
        return;
    }

    if (strcmp(s, "SKIP") == 0) {
        if (s_cal_state == CAL_WAIT_CENTER) {
            printf("CAL> centro pulado\n");
            cal_advance();
        } else {
            printf("CAL> este canto e obrigatorio. Coloque a bola e digite OK.\n");
        }
        return;
    }

    if (strcmp(s, "OK") == 0 || strcmp(s, "NEXT") == 0) {
        bool ok = false;
        switch (s_cal_state) {
        case CAL_WAIT_TL:     ok = cal_capture(&s_corners[COR_TL], "SUP-ESQ (TL)"); break;
        case CAL_WAIT_TR:     ok = cal_capture(&s_corners[COR_TR], "SUP-DIR (TR)"); break;
        case CAL_WAIT_BR:     ok = cal_capture(&s_corners[COR_BR], "INF-DIR (BR)"); break;
        case CAL_WAIT_BL:     ok = cal_capture(&s_corners[COR_BL], "INF-ESQ (BL)"); break;
        case CAL_WAIT_CENTER: ok = cal_capture(&s_center,         "CENTRO");       break;
        default: break;
        }
        if (ok) cal_advance();
        return;
    }

    printf("CAL> comandos: OK (captura) | SKIP (pula) | CANCELAR (aborta)\n");
}

/* ── Calibração de curso dos motores (modo guiado STEPPER) ────────────────── */

static void stepper_set_speeds(float spd, float acc)
{
    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_set_max_speed(i, spd);
        stepper_set_acceleration(i, acc);
    }
}

static void stepper_print(void)
{
    printf("STEPPER> altura ~%.0f mm", s_jog_hz);
    if (s_hz_min_set) printf("  | MIN=%.0f", s_hz_min);
    if (s_hz_max_set) printf("  | MAX=%.0f", s_hz_max);
    printf("\n");
}

static void stepper_start(void)
{
    s_ellipse    = false;
    s_pidauto    = false;
    s_jog_hz     = s_work_hz;
    s_hz_min_set = false;
    s_hz_max_set = false;
    s_stepper    = true;
    pid_reset(&s_pid_x);
    pid_reset(&s_pid_y);
    stepper_set_speeds(SPEED_HOME, ACCEL_HOME);   /* devagar, para nao forcar */
    printf("STEPPER> Calibracao de CURSO dos motores. Mesa NIVELADA, sobe/desce devagar.\n");
    printf("STEPPER> SOBE (+1mm, ou SIM) | DESCE (-1mm) | MINIMO | MAXIMO | MEIO | SALVAR | CANCELAR\n");
    stepper_print();
}

static void stepper_finish(bool save)
{
    if (save) {
        s_work_hz = (s_hz_min + s_hz_max) / 2.0;
        cal_steplim_t lim = { (float)s_hz_min, (float)s_hz_max };
        cal_store_save_steplim(&lim);
        printf("STEPPER> SALVO. MIN=%.0f  MAX=%.0f  MEIO=%.0f mm. O startup usara esse curso.\n",
               s_hz_min, s_hz_max, s_work_hz);
    } else {
        printf("STEPPER> CANCELADO (limites nao alterados).\n");
    }
    s_stepper = false;
    stepper_set_speeds(SPEED_BALANCE, ACCEL_BALANCE);
    /* o laco leva a mesa para s_work_hz (nivelada) automaticamente */
}

static void stepper_step(const char *s)
{
    if (strcmp(s, "CANCELAR") == 0 || strcmp(s, "CANCEL") == 0) { stepper_finish(false); return; }

    if (strcmp(s, "SALVAR") == 0 || strcmp(s, "SAVE") == 0) {
        if (s_hz_min_set && s_hz_max_set && s_hz_min < s_hz_max) stepper_finish(true);
        else printf("STEPPER> defina MINIMO e MAXIMO (com MIN < MAX) antes de SALVAR\n");
        return;
    }

    if (strcmp(s, "SOBE") == 0 || strcmp(s, "SUBIR") == 0 ||
        strcmp(s, "SIM")  == 0 || strcmp(s, "S") == 0) {
        s_jog_hz += STEPPER_JOG_MM;
        if (s_jog_hz > HZ_ABS_MAX) s_jog_hz = HZ_ABS_MAX;
        printf("STEPPER> subiu. pode subir mais? SIM | DESCE | MINIMO | MAXIMO | SALVAR\n");
        stepper_print();
        return;
    }
    if (strcmp(s, "DESCE") == 0 || strcmp(s, "DESCER") == 0 || strcmp(s, "D") == 0) {
        s_jog_hz -= STEPPER_JOG_MM;
        if (s_jog_hz < HZ_ABS_MIN) s_jog_hz = HZ_ABS_MIN;
        stepper_print();
        return;
    }
    if (strcmp(s, "NAO") == 0) {
        printf("STEPPER> ok, parado. Marque MINIMO/MAXIMO, ou DESCE/SOBE.\n");
        stepper_print();
        return;
    }
    if (strcmp(s, "MINIMO") == 0 || strcmp(s, "MIN") == 0) {
        s_hz_min = s_jog_hz; s_hz_min_set = true;
        printf("STEPPER> MINIMO marcado em ~%.0f mm (ponto que bate na mesa)\n", s_hz_min);
        stepper_print();
        return;
    }
    if (strcmp(s, "MAXIMO") == 0 || strcmp(s, "MAX") == 0) {
        s_hz_max = s_jog_hz; s_hz_max_set = true;
        printf("STEPPER> MAXIMO marcado em ~%.0f mm (limite de subida)\n", s_hz_max);
        stepper_print();
        return;
    }
    if (strcmp(s, "MEIO") == 0) {
        if (s_hz_min_set && s_hz_max_set) {
            s_jog_hz = (s_hz_min + s_hz_max) / 2.0;
            printf("STEPPER> indo ao MEIO ~%.0f mm\n", s_jog_hz);
        } else {
            printf("STEPPER> defina MIN e MAX primeiro\n");
        }
        stepper_print();
        return;
    }
    printf("STEPPER> use: SOBE | DESCE | MINIMO | MAXIMO | MEIO | SALVAR | CANCELAR\n");
}

/* ── Auto-tune de PID por relé (Åström–Hägglund + Ziegler–Nichols) ────────── */

static void pidauto_finish(void)
{
    s_pidauto = false;
    if (s_pa_meas <= 0) { printf("PIDAUTO> sem medidas — nada alterado.\n"); return; }
    float Tu = 2.0f * (s_pa_sum_half / (float)s_pa_meas);   /* período completo (s) */
    float a  = s_pa_sum_amp / (float)s_pa_meas;             /* amplitude (mm) */
    if (a < 1.0f || Tu < 0.05f) {
        printf("PIDAUTO> oscilacao fraca (amp=%.1fmm, Tu=%.2fs) — nada alterado.\n",
               (double)a, (double)Tu);
        return;
    }
    float Ku = 4.0f * PA_H / (3.14159265f * a);             /* ganho critico (rad/mm) */
    float kp = 0.6f   * Ku;                                 /* Ziegler–Nichols PID */
    float ki = 1.2f   * Ku / Tu;
    float kd = 0.075f * Ku * Tu;
    cmd_set_all_gains(kp, ki, kd);
    printf("PIDAUTO> RESULTADO: Ku=%.4g  Tu=%.3fs\n", (double)Ku, (double)Tu);
    printf("PIDAUTO> Ziegler-Nichols -> Kp=%.4g  Ki=%.4g  Kd=%.4g  (APLICADOS)\n",
           (double)kp, (double)ki, (double)kd);
    printf("PIDAUTO> teste a bola; se gostar, grave com 'PID SAVE'.\n");
    printf("PIDAUTO> se oscilar demais: KD <maior>  ou  KI <menor>.\n");
}

static void pidauto_start(void)
{
    if (!s_snap_touched) {
        printf("PIDAUTO> coloque a bola perto do CENTRO e digite PIDAUTO de novo.\n");
        return;
    }
    s_ellipse = false; s_stepper = false;
    pid_reset(&s_pid_x); pid_reset(&s_pid_y);
    TickType_t now = xTaskGetTickCount();
    s_pa_relay_x = s_pa_relay_y = 0;
    s_pa_prevsign = 0; s_pa_peak = 0.0f;
    s_pa_last_cross = now; s_pa_start = now; s_pa_seen = now;
    s_pa_sum_half = 0.0f; s_pa_sum_amp = 0.0f; s_pa_meas = 0; s_pa_halfcount = 0;
    s_pidauto = true;
    printf("PIDAUTO> Auto-tune por rele. A mesa vai OSCILAR a bola de proposito (~%d ciclos).\n",
           PA_CYCLES);
    printf("PIDAUTO> Mantenha a bola na mesa. Digite CANCELAR para abortar.\n");
}

/* Chamado pelo laço a 50 Hz enquanto s_pidauto. Devolve a inclinação em *nx,*ny. */
static void pidauto_loop(bool touched, float xmm, float ymm, double *nx, double *ny)
{
    TickType_t now = xTaskGetTickCount();
    *nx = 0.0; *ny = 0.0;

    if (!touched) {
        if ((now - s_pa_seen) > pdMS_TO_TICKS(1200)) {
            printf("PIDAUTO> bola perdida — abortado. Recoloque no centro e tente de novo.\n");
            s_pidauto = false;
        }
        return;   /* mesa nivelada enquanto não vê a bola */
    }
    s_pa_seen = now;

    float ex = xmm - (float)SETPOINT_X_MM;
    float ey = ymm - (float)SETPOINT_Y_MM;

    if (fabsf(ex) > PA_SAFE_MM || fabsf(ey) > PA_SAFE_MM) {
        printf("PIDAUTO> bola foi longe demais (diverge) — abortado.\n");
        printf("PIDAUTO> se a mesa EMPURRA a bola para fora, inverta: SX 1 e/ou SY 1, e repita.\n");
        s_pidauto = false;
        return;
    }
    if ((now - s_pa_start) > pdMS_TO_TICKS((int)(PA_TIMEOUT_S * 1000))) {
        printf("PIDAUTO> tempo esgotado sem oscilacao clara — abortado.\n");
        s_pidauto = false;
        return;
    }

    /* Relé com histerese, realimentação negativa (mesmo sinal do controlador). */
    if      (ex >  PA_HYST) s_pa_relay_x = +1;
    else if (ex < -PA_HYST) s_pa_relay_x = -1;
    if      (ey >  PA_HYST) s_pa_relay_y = +1;
    else if (ey < -PA_HYST) s_pa_relay_y = -1;
    *nx = (double)(s_sign_x * PA_H * (float)s_pa_relay_x);
    *ny = (double)(s_sign_y * PA_H * (float)s_pa_relay_y);

    /* Medição no eixo X: pico e cruzamento por zero do erro. */
    if (fabsf(ex) > s_pa_peak) s_pa_peak = fabsf(ex);
    int sgn = (ex > 0.0f) ? 1 : (ex < 0.0f) ? -1 : s_pa_prevsign;
    if (s_pa_prevsign != 0 && sgn != s_pa_prevsign) {
        float half = (float)(now - s_pa_last_cross) * (float)portTICK_PERIOD_MS / 1000.0f;
        s_pa_last_cross = now;
        s_pa_halfcount++;
        if (s_pa_halfcount > PA_WARMUP && half > 0.02f) {
            s_pa_sum_half += half;
            s_pa_sum_amp  += s_pa_peak;
            s_pa_meas++;
            printf("PIDAUTO> %d/%d  meio-periodo=%.2fs  amp=%.1fmm\n",
                   s_pa_meas, PA_CYCLES, (double)half, (double)s_pa_peak);
            if (s_pa_meas >= PA_CYCLES) { s_pa_peak = 0.0f; pidauto_finish(); return; }
        }
        s_pa_peak = 0.0f;
    }
    s_pa_prevsign = sgn;
}

/* ── Parser de linha serial ───────────────────────────────────────────────── */

static void cmd_handle(char *s)
{
    /* converter para maiúsculas */
    for (char *p = s; *p; ++p) *p = (char)toupper((unsigned char)*p);

    float a, b, c;

    /* Em calibração guiada, só escutamos os passos dela (OK/SKIP/CANCELAR). */
    if (s_cal_state != CAL_IDLE) {
        cal_step(s);
        return;
    }
    /* Em calibração de curso (STEPPER), só escutamos os passos dela. */
    if (s_stepper) {
        stepper_step(s);
        return;
    }
    /* Durante o auto-tune, só aceitamos CANCELAR. */
    if (s_pidauto) {
        if (strcmp(s, "CANCELAR") == 0 || strcmp(s, "CANCEL") == 0 ||
            strcmp(s, "PARAR") == 0) {
            s_pidauto = false;
            printf("PIDAUTO> cancelado.\n");
        } else {
            printf("PIDAUTO> em andamento... CANCELAR para abortar.\n");
        }
        return;
    }

    /* Mostrar / esconder as leituras da tela (telemetria).
     *   SHOW / START  -> liga      HIDDEN / HIDE / STOP -> desliga          */
    if (strcmp(s, "SHOW") == 0 || strcmp(s, "START") == 0) {
        s_telem_enabled = true;
        printf("TELEM,ON\n");
        return;
    }
    else if (strcmp(s, "HIDDEN") == 0 || strcmp(s, "HIDE") == 0 ||
             strcmp(s, "STOP") == 0) {
        s_telem_enabled = false;
        printf("TELEM,OFF\n");
        return;
    }

    /* PID (sozinho) -> mostra os ganhos atuais e como mudá-los.            */
    if (strcmp(s, "PID") == 0) {
        cmd_pid_menu();
        return;
    }

    /* CAL / CALIBRAR (sozinho) -> inicia a calibração guiada dos 4 cantos. */
    if (strcmp(s, "CAL") == 0 || strcmp(s, "CALIBRAR") == 0 ||
        strcmp(s, "CALIBRATE") == 0) {
        cal_start();
        return;
    }

    /* STEPPER -> calibracao de curso dos motores (achar MINIMO/MAXIMO). */
    if (strcmp(s, "STEPPER") == 0 || strcmp(s, "MOTOR") == 0) {
        stepper_start();
        return;
    }

    /* PIDAUTO -> auto-tune do PID por rele (oscila a bola e calcula os ganhos). */
    if (strcmp(s, "PIDAUTO") == 0 || strcmp(s, "AUTOPID") == 0) {
        pidauto_start();
        return;
    }

    /* ELIPSE -> liga/desliga o modo de teste (mesa varre uma elipse).
     * PARAR -> desliga.  Não interfere no controle PID normal.            */
    if (strcmp(s, "ELIPSE") == 0 || strcmp(s, "ELLIPSE") == 0) {
        s_ellipse = !s_ellipse;
        if (s_ellipse) {
            s_stepper = false;
            s_pidauto = false;
            s_ellipse_phase = 0.0;
            pid_reset(&s_pid_x);
            pid_reset(&s_pid_y);
            printf("ELIPSE,ON  (digite ELIPSE ou PARAR para sair)\n");
        } else {
            printf("ELIPSE,OFF\n");
        }
        return;
    }
    if (strcmp(s, "PARAR") == 0) {
        s_ellipse = false;
        printf("ELIPSE,OFF\n");
        return;
    }

    /* SETCAL <xmin> <xmax> <ymin> <ymax> <flip_x> <flip_y> <swap_xy>
     * Comando enviado pelo calibrate.py / PIDSimba.py para aplicar a
     * calibracao calculada no PC diretamente, sem precisar de rebuild.
     * Diferente de "CAL TL/TR..." que captura o toque atual do hardware,
     * SETCAL aceita os valores crus calculados externamente. */
    int ixmin, ixmax, iymin, iymax, ifx, ify, isw;
    if (sscanf(s, "SETCAL %d %d %d %d %d %d %d",
               &ixmin, &ixmax, &iymin, &iymax, &ifx, &ify, &isw) == 7) {
        touch_set_cal(ixmin, ixmax, iymin, iymax, ifx, ify, isw);
        cmd_print_cal();
        return;
    }

    /* PID SAVE — deve vir antes do "PID %f %f %f" para não haver conflito */
    if (strncmp(s, "PID SAVE", 8) == 0) {
        cal_pid_t p = { s_pid_x.kp, s_pid_x.ki, s_pid_x.kd };
        cal_store_save_pid(&p);
        printf("SAVED,pid\n");
        cmd_print_gains();
    }
    /* Ajuste de ganhos PID:
     *   PID <kp> <ki> <kd>   — ajusta os três de uma vez
     *   KP <v> | KI <v> | KD <v> — ajusta um por vez
     *   ?                    — consulta ganhos e sinais
     *   SX <v> | SY <v>      — sinal do controle por eixo (+1/-1)
     */
    else if (sscanf(s, "PID %f %f %f", &a, &b, &c) == 3) {
        cmd_set_all_gains(a, b, c);
        cmd_print_gains();
    } else if (sscanf(s, "KP %f", &a) == 1) {
        s_pid_x.kp = a; s_pid_y.kp = a; cmd_print_gains();
    } else if (sscanf(s, "KI %f", &a) == 1) {
        s_pid_x.ki = a; s_pid_y.ki = a; cmd_print_gains();
    } else if (sscanf(s, "KD %f", &a) == 1) {
        s_pid_x.kd = a; s_pid_y.kd = a; cmd_print_gains();
    } else if (sscanf(s, "SX %f", &a) == 1) {
        s_sign_x = (a < 0.0f) ? -1.0f : +1.0f; cmd_print_signs();
    } else if (sscanf(s, "SY %f", &a) == 1) {
        s_sign_y = (a < 0.0f) ? -1.0f : +1.0f; cmd_print_signs();
    }
    /* Calibração da mesa — CAL <subcomando> */
    else if (strncmp(s, "CAL ", 4) == 0) {
        cmd_handle_cal(s + 4);
    }
    else if (s[0] == '?') {
        cmd_print_gains();
        cmd_print_signs();
        cmd_print_cal();
    }
    else {
        /* comando desconhecido: ajuda curta (ignora linhas vazias) */
        if (s[0] != '\0') {
            printf("ERR,desconhecido: '%s'\n", s);
            printf("Comandos: SHOW | HIDDEN | PID | CAL | SX <v> | SY <v> | ?\n");
        }
    }
}

/* ── Task serial (leitura UART em background) ─────────────────────────────── */

static void cmd_task(void *arg)
{
    /* Lê do stdin do console (USB-Serial-JTAG, o conector USB nativo). Assim os
     * comandos digitados no monitor chegam de fato ao firmware. */
    char line[64];
    int  n = 0;
    for (;;) {
        int ci = fgetc(stdin);
        if (ci == EOF) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        char ch = (char)ci;
        if (ch == '\n' || ch == '\r') {
            if (n > 0) { line[n] = '\0'; cmd_handle(line); n = 0; }
        } else if (n < (int)sizeof(line) - 1) {
            line[n++] = ch;
        }
    }
}

/* ── Cinemática → motores ─────────────────────────────────────────────────── */

/* Move os 3 motores para realizar a inclinação (nx,ny) a uma altura hz. */
static void apply_tilt_hz(double hz, double nx, double ny, double thoff[3])
{
    long pos[STEPPER_COUNT];
    for (int i = 0; i < STEPPER_COUNT; i++) {
        double theta = machine_theta(&s_machine, i, hz, nx, ny);
        if (!isfinite(theta)) theta = s_ang_orig;
        thoff[i] = theta;
        pos[i] = lround((s_ang_orig - theta) * ANG_TO_STEP);
    }
    steppers_move_to_all(pos);
}

/* Versão usual: usa a altura de trabalho calibrada (s_work_hz). */
static void apply_tilt(double nx, double ny, double thoff[3])
{
    apply_tilt_hz(s_work_hz, nx, ny, thoff);
}

/* ── Entry point ──────────────────────────────────────────────────────────── */

void control_run(void)
{
    ESP_LOGI(TAG, "Ball Balancer — Control Mode @ %d Hz", LOOP_HZ);

    /* NVS deve ser inicializado antes de qualquer load/save */
    cal_store_init();

    machine_init(&s_machine, GEO_D, GEO_E, GEO_F, GEO_G);

    /* Carrega ganhos PID do NVS; usa defaults de compilação se não há nada salvo */
    cal_pid_t saved_pid;
    if (cal_store_load_pid(&saved_pid)) {
        pid_init(&s_pid_x, saved_pid.kp, saved_pid.ki, saved_pid.kd,
                 -TILT_LIMIT, TILT_LIMIT);
        pid_init(&s_pid_y, saved_pid.kp, saved_pid.ki, saved_pid.kd,
                 -TILT_LIMIT, TILT_LIMIT);
    } else {
        pid_init(&s_pid_x, KP, KI, KD, -TILT_LIMIT, TILT_LIMIT);
        pid_init(&s_pid_y, KP, KI, KD, -TILT_LIMIT, TILT_LIMIT);
    }

    s_ang_orig = machine_theta(&s_machine, LEG_A, GEO_HZ, 0.0, 0.0);
    ESP_LOGI(TAG, "ang_orig = %.3f deg  (steps/rev=%d)", s_ang_orig, STEPS_PER_REV);

    touch_init();

    /* Carrega calibração do touch do NVS se disponível */
    cal_touch_t saved_touch;
    if (cal_store_load_touch(&saved_touch)) {
        touch_set_cal(saved_touch.x_raw_min, saved_touch.x_raw_max,
                      saved_touch.y_raw_min, saved_touch.y_raw_max,
                      saved_touch.flip_x, saved_touch.flip_y, saved_touch.swap_xy);
    }

    steppers_init();

    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_set_max_speed(i, SPEED_HOME);
        stepper_set_acceleration(i, ACCEL_HOME);
    }
    steppers_enable(true);
    vTaskDelay(pdMS_TO_TICKS(200));

    double thoff[3];

    /* Carrega o curso calibrado (STEPPER) do NVS, se existir. */
    cal_steplim_t lim;
    bool have_lim = cal_store_load_steplim(&lim);
    if (have_lim && lim.hz_min < lim.hz_max) {
        s_hz_min = lim.hz_min; s_hz_max = lim.hz_max;
        s_hz_min_set = s_hz_max_set = true;
        s_work_hz = (s_hz_min + s_hz_max) / 2.0;
    } else {
        s_work_hz = GEO_HZ;     /* sem calibração: comportamento padrão */
    }

    if (have_lim && lim.hz_min < lim.hz_max) {
        /* Startup calibrado: SOBE -> DESCE -> para no MEIO, dentro do curso. */
        ESP_LOGI(TAG, "Startup: curso %.0f..%.0f mm, meio=%.0f",
                 s_hz_min, s_hz_max, s_work_hz);
        apply_tilt_hz(s_hz_max,  0.0, 0.0, thoff);  /* sobe ao maximo */
        steppers_run_to_position_blocking();
        vTaskDelay(pdMS_TO_TICKS(300));
        apply_tilt_hz(s_hz_min,  0.0, 0.0, thoff);  /* desce ao minimo */
        steppers_run_to_position_blocking();
        vTaskDelay(pdMS_TO_TICKS(300));
        apply_tilt_hz(s_work_hz, 0.0, 0.0, thoff);  /* para no meio */
        steppers_run_to_position_blocking();
        vTaskDelay(pdMS_TO_TICKS(300));
    } else {
        /* Sem calibração de curso: homing antigo (sobe pouco e nivela). */
        long pos[STEPPER_COUNT];
        for (int i = 0; i < STEPPER_COUNT; i++) {
            double theta_lift = machine_theta(&s_machine, i,
                                              GEO_HZ + HOMING_LIFT_MM, 0.0, 0.0);
            if (!isfinite(theta_lift)) theta_lift = s_ang_orig;
            pos[i] = lround((s_ang_orig - theta_lift) * ANG_TO_STEP);
        }
        steppers_move_to_all(pos);
        steppers_run_to_position_blocking();
        vTaskDelay(pdMS_TO_TICKS(300));
        apply_tilt(0.0, 0.0, thoff);   /* nivela em s_work_hz (=GEO_HZ) */
        steppers_run_to_position_blocking();
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_set_max_speed(i, SPEED_BALANCE);
        stepper_set_acceleration(i, ACCEL_BALANCE);
    }

    printf("# ================================================================\n");
    printf("# Ball Balancer — ESP32-S3  |  3RPS  |  50 Hz\n");
    printf("# ================================================================\n");
    printf("# LAYOUT DOS MOTORES (vista de cima):\n");
    printf("#          CIMA / FRENTE\n");
    printf("#        Motor A (STEP=15 DIR=16)\n");
    printf("#            |\n");
    printf("#   ESQUERDA [MESA] DIREITA (cabo FPC)\n");
    printf("#           / \\  \n");
    printf("# MotorC(40/41)  MotorB(17/18)\n");
    printf("#   BAIXO-ESQ      BAIXO-DIR\n");
    printf("# ----------------------------------------------------------------\n");
    printf("# COORDENADAS DA MESA:\n");
    printf("#   x_mm = 0 -> borda ESQUERDA  |  x_mm = %.0f -> DIREITA\n",
           (double)TOUCH_WIDTH_MM);
    printf("#   y_mm = 0 -> borda SUPERIOR  |  y_mm = %.0f -> INFERIOR\n",
           (double)TOUCH_HEIGHT_MM);
    printf("#   Setpoint: centro  (%.1f, %.1f) mm\n",
           (double)SETPOINT_X_MM, (double)SETPOINT_Y_MM);
    printf("# ----------------------------------------------------------------\n");
    printf("# ENABLE: GPIO 10 (ativo LOW)  |  HOMING_LIFT: %.0f mm\n",
           HOMING_LIFT_MM);
    printf("# ----------------------------------------------------------------\n");
    printf("# Motor inversão (steppers.h): MOT_A=%d  MOT_B=%d  MOT_C=%d\n",
           MOTOR_A_INVERT, MOTOR_B_INVERT, MOTOR_C_INVERT);
    printf("#  Se um motor subir/descer ao contrário no homing, mude o flag\n");
    printf("# ----------------------------------------------------------------\n");
    printf("# COMANDOS PELO TERMINAL SERIAL:\n");
    printf("#   SHOW           -> mostra as leituras da tela (x,y em mm)\n");
    printf("#   HIDDEN         -> esconde as leituras\n");
    printf("#   PID            -> mostra Kp/Ki/Kd atuais e como mudar\n");
    printf("#   PID <kp> <ki> <kd>  | KP <v> | KI <v> | KD <v> | PID SAVE\n");
    printf("#   CAL            -> calibracao guiada (4 cantos + centro)\n");
    printf("#                     responda OK / SKIP / CANCELAR ; no fim SALVAR\n");
    printf("#   ELIPSE         -> teste: mesa varre uma elipse (PARAR p/ sair)\n");
    printf("#   STEPPER        -> calibra o CURSO dos motores (MINIMO/MAXIMO)\n");
    printf("#                     SOBE|DESCE 1mm ; MINIMO ; MAXIMO ; SALVAR\n");
    printf("#   PIDAUTO        -> auto-tune do PID (oscila a bola e calcula)\n");
    printf("#   SX <v> | SY <v>    -> sinal do controle por eixo (+1/-1)\n");
    printf("#   ?              -> exibe ganhos, sinais e calibracao\n");
    printf("# ================================================================\n");
    printf("# Pronto. Digite SHOW para ver a tela, ou CAL para calibrar.\n");
    printf("# ================================================================\n");

    xTaskCreate(cmd_task, "pid_cmd", 3072, NULL, 5, NULL);

    cmd_print_gains();
    cmd_print_signs();
    cmd_print_cal();

    bool detected = false;          /* estado ACTIVE (bola sob controle) */
    int  presence = 0, absence = 0; /* contadores de debounce */
    double last_nx = 0.0, last_ny = 0.0;  /* última inclinação (segura queda curta) */
    unsigned loopn = 0;             /* contador p/ throttle da telemetria */
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        touch_pos_t p;
        bool touched = touch_read(&p);
        loopn++;

        /* Atualiza snapshot compartilhado com o cmd_task */
        s_snap_touched = touched;
        if (touched) {
            s_snap_x_raw = p.x_raw;
            s_snap_y_raw = p.y_raw;
        }

        /* Mede o tempo real decorrido (compensar printf delays no PID) */
        TickType_t now = xTaskGetTickCount();
        float dt_real = (float)(now - last_wake) * 0.001f;  /* ms → s */
        if (dt_real <= 0.0f) dt_real = LOOP_DT;
        if (dt_real > 0.1f) dt_real = LOOP_DT;  /* proteção contra overflow */

        double nx = 0.0, ny = 0.0;

        if (s_stepper) {
            /* Calibração de curso: mesa NIVELADA na altura sendo ajustada. */
            apply_tilt_hz(s_jog_hz, 0.0, 0.0, thoff);
            detected = false;
        } else if (s_pidauto) {
            /* Auto-tune: o relé comanda a inclinação (oscila a bola). */
            pidauto_loop(touched, p.x_mm, p.y_mm, &nx, &ny);
            apply_tilt(nx, ny, thoff);
            detected = false;
        } else {
            if (s_ellipse) {
                /* Teste: a mesa descreve uma elipse suave (sem PID). */
                s_ellipse_phase += ELLIPSE_TWO_PI * ELLIPSE_HZ * dt_real;
                if (s_ellipse_phase > ELLIPSE_TWO_PI) s_ellipse_phase -= ELLIPSE_TWO_PI;
                nx = ELLIPSE_AMP_X * cos(s_ellipse_phase);
                ny = ELLIPSE_AMP_Y * sin(s_ellipse_phase);
                detected = false;
            } else if (s_calibrating) {
                /* Durante a calibração a mesa fica NIVELADA (nx=ny=0): a bola
                 * fica onde você a coloca, sem o PID empurrá-la para o centro. */
                detected = false;
            } else {
                /* ── Controle normal com presença DEBOUNCED ────────────────
                 * Só controla com a bola realmente detectada; sem bola, IDLE
                 * (mesa nivelada na posição padrão), aguardando o toque. */
                if (touched) { presence++; absence = 0; }
                else         { absence++;  presence = 0; }

                if (!detected && presence >= PRESENCE_ON) {
                    detected = true;
                    pid_reset(&s_pid_x);
                    pid_reset(&s_pid_y);
                    printf("STATE,ACTIVE (bola detectada)\n");
                } else if (detected && absence >= PRESENCE_OFF) {
                    detected = false;
                    pid_reset(&s_pid_x);
                    pid_reset(&s_pid_y);
                    printf("STATE,IDLE (sem bola)\n");
                }

                if (detected) {
                    if (touched) {
                        float ox = pid_update(&s_pid_x, p.x_mm, SETPOINT_X_MM, dt_real);
                        float oy = pid_update(&s_pid_y, p.y_mm, SETPOINT_Y_MM, dt_real);
                        nx = s_sign_x * ox;
                        ny = s_sign_y * oy;
                        last_nx = nx; last_ny = ny;   /* lembra p/ queda curta */
                    } else {
                        nx = last_nx; ny = last_ny;   /* segura durante a queda curta */
                    }
                }
                /* se !detected: nx=ny=0 -> IDLE (mesa nivelada na posicao padrao) */
            }
            apply_tilt(nx, ny, thoff);
        }

        if (s_pidauto) {
            /* O proprio pidauto_loop ja imprime o progresso — nao spamar aqui. */
        }
        else if (s_stepper) {
            /* Status throttled (~3 Hz) da calibração de curso. */
            TickType_t tn = xTaskGetTickCount();
            if (tn - s_cal_live_last >= pdMS_TO_TICKS(300)) {
                s_cal_live_last = tn;
                printf("STEPPER,hz=%.0f,thA=%.1f,thB=%.1f,thC=%.1f\n",
                       s_jog_hz, thoff[0], thoff[1], thoff[2]);
            }
        }
        else if (s_ellipse) {
            /* Status throttled (~3 Hz) do movimento de teste. */
            TickType_t tn = xTaskGetTickCount();
            if (tn - s_cal_live_last >= pdMS_TO_TICKS(300)) {
                s_cal_live_last = tn;
                printf("ELIPSE,nx=%.3f,ny=%.3f,thA=%.1f,thB=%.1f,thC=%.1f\n",
                       nx, ny, thoff[0], thoff[1], thoff[2]);
            }
        }
        else if (s_calibrating) {
            /* Stream LIVE enxuto (~2,5 Hz) para acompanhar a leitura ao colocar
             * a bola em cada ponto. Usa a calibração vigente (em mm). */
            TickType_t tn = xTaskGetTickCount();
            if (tn - s_cal_live_last >= pdMS_TO_TICKS(400)) {
                s_cal_live_last = tn;
                if (touched) {
                    printf("LIVE,%ld,%ld,%.1f,%.1f\n",
                           (long)p.x_raw, (long)p.y_raw, p.x_mm, p.y_mm);
                } else {
                    printf("LIVE,%ld,%ld,sem_toque\n",
                           (long)p.x_raw, (long)p.y_raw);
                }
            }
        }
        /* Telemetria normal — só fora da calibração e quando habilitada (SHOW).
         * A 100 Hz, imprime em ciclos alternados (~50 Hz) para não congestionar
         * a serial e não introduzir atraso no laço de controle. */
        else if (s_telem_enabled && (loopn & 1u)) {
            if (touched) {
                printf("POS,%ld,%ld,%.1f,%.1f\n",
                       (long)p.x_raw, (long)p.y_raw, p.x_mm, p.y_mm);
            } else {
                printf("NOTOUCH,%ld,%ld\n", (long)p.x_raw, (long)p.y_raw);
            }
            printf("CTRL,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.2f,%.2f,%.2f\n",
                   touched ? p.x_mm : -1.0f, touched ? p.y_mm : -1.0f,
                   SETPOINT_X_MM, SETPOINT_Y_MM, nx, ny,
                   thoff[0], thoff[1], thoff[2]);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}
