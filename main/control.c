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

/* Carência no startup: após o homing, a mesa fica NIVELADA por este tempo antes
 * de permitir o controle. Deixa a leitura da tela estabilizar (o ruído elétrico
 * do homing desloca o ponto-preso e dava um falso "tem bola" momentâneo). */
#define STARTUP_GRACE_MS  1500

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

/* Viés de nível (TRIM): inclinação constante somada à saída para compensar a
 * base torta. Aprendido do integral em regime; persiste no NVS. Aplicado também
 * em IDLE, para a mesa já esperar a bola nivelada em relação à gravidade. */
static float     s_trim_x = 0.0f;
static float     s_trim_y = 0.0f;

/* Último toque lido pelo loop principal; usado pelo cmd_task para capturar
 * cantos. Campos 32-bit são atômicos em ARM — sem mutex para leitura simples. */
static volatile int32_t s_snap_x_raw   = 0;
static volatile int32_t s_snap_y_raw   = 0;
static volatile bool    s_snap_touched = false;

/* Telemetria habilitada apenas entre START/STOP */
static volatile bool    s_telem_enabled = false;

/* Comando ERROR: imprime o erro (distância do centro) a cada 1 s */
static volatile bool    s_show_error = false;
static TickType_t       s_err_last   = 0;

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
 * Máquina de estados conduzida pelos comandos OK / CANCELAR. Enquanto
 * ativa (s_calibrating), o laço mantém a mesa NIVELADA (não balanceia), para a
 * bola ficar onde você a coloca, e emite um stream LIVE enxuto a ~2,5 Hz.
 */
typedef enum {
    CAL_IDLE, CAL_WAIT_TL, CAL_WAIT_TR, CAL_WAIT_BR, CAL_WAIT_BL,
    CAL_CONFIRM
} cal_state_t;
static volatile cal_state_t s_cal_state    = CAL_IDLE;
static volatile bool        s_calibrating  = false;
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
 * desse curso, sem forçar. s_level_steps = offset de repouso (em passos).
 */
/* STEPPER agora trabalha em PASSOS (não em altura) — assim não satura na
 * cinemática (geometria placeholder) e você chega ao limite físico real. */
static volatile bool s_stepper      = false;
static long          s_jog_steps    = 0;    /* posição (passos) sendo ajustada */
static long          s_level_steps  = 0;    /* offset de repouso/nível (passos) */
static long          s_step_min     = 0;
static long          s_step_max     = 0;
static bool          s_step_min_set = false;
static bool          s_step_max_set = false;
#define STEP_JOG        13         /* ~1 mm por SOBE/DESCE (≈12,8 passos/mm) */
#define STEP_ABS_LIM    1500       /* trava de segurança (passos) */

/* ── Auto-tune de PID ADAPTATIVO (aprende balanceando) ────────────────────
 * "PIDAUTO": o firmware balança a bola normalmente e usa como PONTUAÇÃO o
 * TEMPO que a bola fica na mesa (quanto mais, melhor). A cada tentativa ele
 * perturba um ganho (Kp ou Kd) por hill-climbing: se a pontuação melhora,
 * mantém e segue na mesma direção; se piora, volta e tenta outra. Detecta
 * quando a bola cai (fim de tentativa) e quando você recoloca (nova tentativa),
 * convergindo para ganhos que NÃO deixam a bola sair. */
static volatile bool s_pidauto = false;       /* tuning adaptativo ligado */
static bool       s_auto_baseline;            /* 1a tentativa = baseline */
static TickType_t s_auto_tstart;              /* início da tentativa atual */
static float      s_auto_errsum;              /* soma do erro p/ média */
static int        s_auto_errn;
static float      s_best_kp, s_best_ki, s_best_kd, s_best_score;
static int        s_auto_move;                /* 0..5: kp+ kp- kd+ kd- ki+ ki- */
static int        s_auto_since;               /* tentativas sem melhora */
static float      s_auto_step;                /* fator multiplicativo dos ganhos */
static int        s_auto_trial;               /* contador de tentativas */
#define AUTO_NMOVES       6       /* kp± kd± ki± */
#define AUTO_TRIAL_MAX_S  5.0f    /* janela de avaliação de cada conjunto */
#define AUTO_SETTLE_S     1.5f    /* ignora o transitório de colocar a bola */
#define AUTO_STEP0        1.25f   /* passo inicial (×/÷ nos ganhos) — gentil */
#define AUTO_STEP_MIN     1.05f   /* passo mínimo (convergência fina) */

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

static void cmd_trim_show(void)
{
    printf("TRIM,nx=%.4f,ny=%.4f rad  (%.2f, %.2f graus)\n",
           (double)s_trim_x, (double)s_trim_y,
           (double)(s_trim_x * 57.29578f), (double)(s_trim_y * 57.29578f));
}

/* Transfere o que o integral acumulou (a inclinação constante que ele descobriu
 * para a base torta) para o viés fixo s_trim, e ZERA o integral. Sem salto: o
 * que sai do integral entra no trim. Faça com a bola equilibrada há ~10-20 s. */
static void cmd_trim_capture(void)
{
    s_trim_x += s_sign_x * (s_pid_x.ki * s_pid_x.integ);
    s_trim_y += s_sign_y * (s_pid_y.ki * s_pid_y.integ);
    s_pid_x.integ = 0.0f;
    s_pid_y.integ = 0.0f;
    /* salvaguarda: nunca deixa o viés sozinho estourar o curso útil */
    if (s_trim_x >  TILT_LIMIT) s_trim_x =  TILT_LIMIT;
    if (s_trim_x < -TILT_LIMIT) s_trim_x = -TILT_LIMIT;
    if (s_trim_y >  TILT_LIMIT) s_trim_y =  TILT_LIMIT;
    if (s_trim_y < -TILT_LIMIT) s_trim_y = -TILT_LIMIT;
    printf("TRIM,capturado do integral\n");
    cmd_trim_show();
}

static void cmd_trim_save(void)
{
    cal_trim_t t = { s_trim_x, s_trim_y };
    cal_store_save_trim(&t);
    printf("SAVED,trim\n");
    cmd_trim_show();
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

    /* Centro derivado dos 4 cantos (ponto médio) — NÃO é medido com a bola,
     * para não enviesar a calibração. Por construção do mapeamento linear ele
     * cai exatamente no centro geométrico (TOUCH_WIDTH/2, TOUCH_HEIGHT/2 mm). */
    int32_t cx_raw = (left_x + right_x) / 2;
    int32_t cy_raw = (top_y  + bot_y ) / 2;
    printf("CAL> centro (dos cantos): raw=(%ld,%ld) -> alvo %.0f,%.0f mm\n",
           (long)cx_raw, (long)cy_raw,
           (double)SETPOINT_X_MM, (double)SETPOINT_Y_MM);
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

static void cmd_print_help(void)
{
    printf("=============== COMANDOS ===============\n");
    printf(" SHOW / HIDDEN   mostra / esconde leituras da tela\n");
    printf(" ERROR           imprime o erro (dist. do centro) a cada 1s (ERROR OFF)\n");
    printf(" TRIM            aprende o nivel da base torta | TRIM SHOW/CLR/SAVE\n");
    printf(" PID             mostra Kp/Ki/Kd e como mudar\n");
    printf(" PID <kp> <ki> <kd> ('-' mantem) | KP/KI/KD <v> | PID SAVE\n");
    printf(" PIDAUTO         auto-tune do PID (oscila a bola)\n");
    printf(" CAL             calibra a tela (4 cantos + centro)\n");
    printf(" STEPPER         calibra o curso dos motores (MINIMO/MAXIMO)\n");
    printf(" ELIPSE          teste: mesa varre uma elipse (PARAR p/ sair)\n");
    printf(" ZERO            marca ponto morto da tela (acumula) e salva\n");
    printf(" ZEROCLR         limpa os pontos mortos\n");
    printf(" SX <v> | SY <v> sinal do controle por eixo (+1/-1)\n");
    printf(" ?               estado atual (ganhos, sinais, calibracao)\n");
    printf(" HELP            esta lista\n");
    printf("========================================\n");
}

static void cmd_pid_menu(void)
{
    printf("----------------------------------------------\n");
    printf(" PID atual:  Kp=%.6g   Ki=%.6g   Kd=%.6g\n",
           (double)s_pid_x.kp, (double)s_pid_x.ki, (double)s_pid_x.kd);
    printf(" Para mudar:  PID <kp> <ki> <kd>   ('-' mantem o atual)\n");
    printf("              ex.: PID - 0.5 0.2  (mantem Kp, muda Ki e Kd)\n");
    printf("              KP <v>   |   KI <v>   |   KD <v>\n");
    printf(" Gravar NVS:  PID SAVE\n");
    printf("----------------------------------------------\n");
}

/* ── Calibração guiada: prompts e máquina de estados ──────────────────────── */

static void cal_print_prompt(void)
{
    switch (s_cal_state) {
    case CAL_WAIT_TL:
        printf("CAL> [1/4] Bola no canto SUPERIOR-ESQUERDO  -> digite OK\n"); break;
    case CAL_WAIT_TR:
        printf("CAL> [2/4] Bola no canto SUPERIOR-DIREITO (lado do cabo) -> OK\n"); break;
    case CAL_WAIT_BR:
        printf("CAL> [3/4] Bola no canto INFERIOR-DIREITO (lado do cabo) -> OK\n"); break;
    case CAL_WAIT_BL:
        printf("CAL> [4/4] Bola no canto INFERIOR-ESQUERDO -> OK\n"); break;
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
    case CAL_WAIT_BL:
        /* 4 cantos prontos: calcula a calibração E o centro a partir deles.
         * O centro NÃO é mais medido com a bola (isso enviesava a calibração);
         * ele é o ponto médio dos 4 cantos, garantido geometricamente. */
        cmd_cal_apply();   /* aplica ao vivo; imprime CAL,... e o centro calculado */
        s_cal_state = CAL_CONFIRM;
        printf("CAL> Calibracao aplicada (centro derivado dos cantos). "
               "Confira o CENTRO no stream LIVE.\n");
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
    s_ellipse     = false;        /* nao roda elipse durante a calibracao */
    s_stepper     = false;
    s_pidauto     = false;
    s_cal_state   = CAL_WAIT_TL;
    s_calibrating = true;
    pid_reset(&s_pid_x);   /* a mesa para de balancear durante a calibração */
    pid_reset(&s_pid_y);
    printf("CAL> Modo CALIBRACAO. A mesa fica NIVELADA (nao balanceia).\n");
    printf("CAL> Orientacao: cabo FPC a DIREITA.  Comandos: OK | CANCELAR\n");
    printf("CAL> So os 4 cantos. O centro e calculado a partir deles.\n");
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
        /* os 4 cantos são todos obrigatórios; o centro é derivado deles */
        printf("CAL> este canto e obrigatorio. Coloque a bola e digite OK.\n");
        return;
    }

    if (strcmp(s, "OK") == 0 || strcmp(s, "NEXT") == 0) {
        bool ok = false;
        switch (s_cal_state) {
        case CAL_WAIT_TL:     ok = cal_capture(&s_corners[COR_TL], "SUP-ESQ (TL)"); break;
        case CAL_WAIT_TR:     ok = cal_capture(&s_corners[COR_TR], "SUP-DIR (TR)"); break;
        case CAL_WAIT_BR:     ok = cal_capture(&s_corners[COR_BR], "INF-DIR (BR)"); break;
        case CAL_WAIT_BL:     ok = cal_capture(&s_corners[COR_BL], "INF-ESQ (BL)"); break;
        default: break;
        }
        if (ok) cal_advance();
        return;
    }

    printf("CAL> comandos: OK (captura canto) | CANCELAR (aborta)\n");
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
    printf("STEPPER> pos ~%ld passos (~%.0f mm)", s_jog_steps, (double)s_jog_steps / 12.8);
    if (s_step_min_set) printf("  | MIN=%ld", s_step_min);
    if (s_step_max_set) printf("  | MAX=%ld", s_step_max);
    printf("\n");
}

static void stepper_start(void)
{
    s_ellipse      = false;
    s_pidauto      = false;
    s_jog_steps    = s_level_steps;     /* começa na altura de repouso atual */
    s_step_min_set = false;
    s_step_max_set = false;
    s_stepper      = true;
    pid_reset(&s_pid_x);
    pid_reset(&s_pid_y);
    stepper_set_speeds(SPEED_HOME, ACCEL_HOME);   /* devagar, para nao forcar */
    printf("STEPPER> Calibracao de CURSO (em passos). Mesa NIVELADA, sobe/desce devagar.\n");
    printf("STEPPER> SOBE (~1mm, ou SIM) | DESCE | MINIMO | MAXIMO | MEIO | SALVAR | CANCELAR\n");
    stepper_print();
}

static void stepper_finish(bool save)
{
    if (save) {
        s_level_steps = (s_step_min + s_step_max) / 2;
        cal_steplim_t lim = { (int32_t)s_step_min, (int32_t)s_step_max };
        cal_store_save_steplim(&lim);
        printf("STEPPER> SALVO. MIN=%ld  MAX=%ld  MEIO=%ld passos. Startup usara esse curso.\n",
               s_step_min, s_step_max, s_level_steps);
    } else {
        printf("STEPPER> CANCELADO (limites nao alterados).\n");
    }
    s_stepper = false;
    stepper_set_speeds(SPEED_BALANCE, ACCEL_BALANCE);
    /* o laco leva a mesa para s_level_steps (nivelada) automaticamente */
}

static void stepper_step(const char *s)
{
    if (strcmp(s, "CANCELAR") == 0 || strcmp(s, "CANCEL") == 0) { stepper_finish(false); return; }

    if (strcmp(s, "SALVAR") == 0 || strcmp(s, "SAVE") == 0) {
        if (s_step_min_set && s_step_max_set && s_step_min < s_step_max) stepper_finish(true);
        else printf("STEPPER> defina MINIMO e MAXIMO (com MIN < MAX) antes de SALVAR\n");
        return;
    }

    if (strcmp(s, "SOBE") == 0 || strcmp(s, "SUBIR") == 0 ||
        strcmp(s, "SIM")  == 0 || strcmp(s, "S") == 0) {
        s_jog_steps += STEP_JOG;
        if (s_jog_steps > STEP_ABS_LIM) s_jog_steps = STEP_ABS_LIM;
        printf("STEPPER> subiu. pode subir mais? SIM | DESCE | MINIMO | MAXIMO | SALVAR\n");
        stepper_print();
        return;
    }
    if (strcmp(s, "DESCE") == 0 || strcmp(s, "DESCER") == 0 || strcmp(s, "D") == 0) {
        s_jog_steps -= STEP_JOG;
        if (s_jog_steps < -STEP_ABS_LIM) s_jog_steps = -STEP_ABS_LIM;
        stepper_print();
        return;
    }
    if (strcmp(s, "NAO") == 0) {
        printf("STEPPER> ok, parado. Marque MINIMO/MAXIMO, ou DESCE/SOBE.\n");
        stepper_print();
        return;
    }
    if (strcmp(s, "MINIMO") == 0 || strcmp(s, "MIN") == 0) {
        s_step_min = s_jog_steps; s_step_min_set = true;
        printf("STEPPER> MINIMO marcado em %ld passos (ponto que bate na mesa)\n", s_step_min);
        stepper_print();
        return;
    }
    if (strcmp(s, "MAXIMO") == 0 || strcmp(s, "MAX") == 0) {
        s_step_max = s_jog_steps; s_step_max_set = true;
        printf("STEPPER> MAXIMO marcado em %ld passos (limite de subida)\n", s_step_max);
        stepper_print();
        return;
    }
    if (strcmp(s, "MEIO") == 0) {
        if (s_step_min_set && s_step_max_set) {
            s_jog_steps = (s_step_min + s_step_max) / 2;
            printf("STEPPER> indo ao MEIO %ld passos\n", s_jog_steps);
        } else {
            printf("STEPPER> defina MIN e MAX primeiro\n");
        }
        stepper_print();
        return;
    }
    printf("STEPPER> use: SOBE | DESCE | MINIMO | MAXIMO | MEIO | SALVAR | CANCELAR\n");
}

/* ── Auto-tune de PID ADAPTATIVO (hill-climbing por tempo na mesa) ────────── */

static float auto_clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* Define os ganhos vivos = melhor + perturbação do movimento atual. */
static void auto_propose(void)
{
    float kp = s_best_kp, ki = s_best_ki, kd = s_best_kd;
    if (!s_auto_baseline) {
        float up = s_auto_step, dn = 1.0f / s_auto_step;
        switch (s_auto_move) {
        case 0: kp *= up; break;
        case 1: kp *= dn; break;
        case 2: kd *= up; break;
        case 3: kd *= dn; break;
        case 4: ki = (ki < 1.0e-5f) ? 2.0e-5f : ki * up; break;   /* ki+ (sai do zero) */
        case 5: ki = ki * dn; if (ki < 1.0e-5f) ki = 0.0f; break; /* ki- (pode zerar) */
        }
    }
    kp = auto_clampf(kp, 1.0e-4f, 1.5e-2f);
    kd = auto_clampf(kd, 1.0e-3f, 6.0e-2f);
    ki = auto_clampf(ki, 0.0f,    1.0e-3f);
    cmd_set_all_gains(kp, ki, kd);
}

static void auto_trial_begin(void)
{
    s_auto_tstart = xTaskGetTickCount();
    s_auto_errsum = 0.0f;
    s_auto_errn   = 0;
}

/* Fecha a tentativa: pontua (tempo na mesa + centragem), atualiza o melhor por
 * hill-climbing e propõe os próximos ganhos. fell=true se a bola caiu. */
static void auto_trial_end(bool fell)
{
    float t = (float)(xTaskGetTickCount() - s_auto_tstart) *
              (float)portTICK_PERIOD_MS / 1000.0f;
    float meanerr = (s_auto_errn > 0) ? (s_auto_errsum / (float)s_auto_errn) : 99.0f;

    /* PONTUAÇÃO (maior = melhor):
     *  - SOBREVIVEU a janela -> score = -erro_medio (menos erro = melhor).
     *  - CAIU -> fortemente penalizado (sempre pior que qualquer sobrevivente),
     *    ordenado por quanto durou. Assim o sinal é o ERRO (contínuo, comparável)
     *    e não o tempo (ruidoso), e cair nunca é "premiado". */
    float score = fell ? (-200.0f + 5.0f * t) : (-meanerr);
    s_auto_trial++;

    float cur_kp = s_pid_x.kp, cur_ki = s_pid_x.ki, cur_kd = s_pid_x.kd;
    bool improved = (s_best_score <= -1.0e8f) || (score > s_best_score + 0.3f);

    if (improved) {
        s_best_kp = cur_kp; s_best_ki = cur_ki; s_best_kd = cur_kd;
        s_best_score = score;
        s_auto_since = 0;                 /* mantém o mesmo movimento (momentum) */
        printf("PIDAUTO> #%d %s t=%.1fs err=%.0fmm score=%.1f -> MELHOR  Kp=%.2e Ki=%.2e Kd=%.2e\n",
               s_auto_trial, fell ? "caiu" : "ok", (double)t, (double)meanerr, (double)score,
               (double)cur_kp, (double)cur_ki, (double)cur_kd);
    } else {
        s_auto_move = (s_auto_move + 1) % AUTO_NMOVES;
        s_auto_since++;
        if (s_auto_since >= AUTO_NMOVES) {   /* ciclo completo sem melhora -> passo menor */
            s_auto_step = 1.0f + (s_auto_step - 1.0f) * 0.6f;
            if (s_auto_step < AUTO_STEP_MIN) s_auto_step = AUTO_STEP_MIN;
            s_auto_since = 0;
        }
        printf("PIDAUTO> #%d %s t=%.1fs err=%.0fmm score=%.1f (melhor=%.1f) -> tenta outro\n",
               s_auto_trial, fell ? "caiu" : "ok", (double)t, (double)meanerr,
               (double)score, (double)s_best_score);
    }
    s_auto_baseline = false;
    auto_propose();
}

static void auto_start(void)
{
    if (!s_snap_touched) {
        printf("PIDAUTO> coloque a bola na mesa e digite PIDAUTO de novo.\n");
        return;
    }
    s_ellipse = false; s_stepper = false;
    /* Parte SEMPRE de um baseline AMORTECIDO conhecido (defaults do firmware),
     * que segura a bola. Assim as perturbações gentis não a derrubam e dá para
     * medir o erro continuamente — não importa de que ganhos a sessão começou. */
    s_best_kp = KP; s_best_ki = KI; s_best_kd = KD;
    s_best_score = -1.0e9f;
    s_auto_move = 0; s_auto_since = 0; s_auto_step = AUTO_STEP0;
    s_auto_trial = 0; s_auto_baseline = true;
    s_pidauto = true;
    pid_reset(&s_pid_x); pid_reset(&s_pid_y);
    auto_propose();          /* aplica o baseline seguro */
    auto_trial_begin();      /* cronometra já (a bola pode estar na mesa) */
    printf("PIDAUTO> Tuning ADAPTATIVO. Parte de ganhos seguros e REFINA sozinho.\n");
    printf("PIDAUTO> Deixe a bola na mesa — ele testa um ajuste a cada %.0fs sem derrubar.\n",
           (double)AUTO_TRIAL_MAX_S);
    printf("PIDAUTO> Quanto MENOR o 'err', melhor. PARAR aplica o melhor; 'PID SAVE' grava.\n");
}

static void auto_stop(void)
{
    cmd_set_all_gains(s_best_kp, s_best_ki, s_best_kd);
    s_pidauto = false;
    printf("PIDAUTO> encerrado. MELHOR Kp=%.3e Ki=%.3e Kd=%.3e (score=%.1f).\n",
           (double)s_best_kp, (double)s_best_ki, (double)s_best_kd, (double)s_best_score);
    printf("PIDAUTO> 'PID SAVE' para gravar no NVS.\n");
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
    /* Durante o auto-tune adaptativo, PARAR encerra (aplica o melhor). */
    if (s_pidauto) {
        if (strcmp(s, "CANCELAR") == 0 || strcmp(s, "CANCEL") == 0 ||
            strcmp(s, "PARAR") == 0) {
            auto_stop();
        } else {
            printf("PIDAUTO> tunando... deixe a bola; PARAR para encerrar.\n");
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

    /* HELP -> lista de comandos. */
    if (strcmp(s, "HELP") == 0 || strcmp(s, "AJUDA") == 0) {
        cmd_print_help();
        return;
    }

    /* ERROR -> liga/desliga a impressão do erro (a cada 1 s). ERROR OFF desliga. */
    if (strcmp(s, "ERROR OFF") == 0 || strcmp(s, "ERRO OFF") == 0) {
        s_show_error = false;
        printf("ERROR,OFF\n");
        return;
    }
    if (strcmp(s, "ERROR") == 0 || strcmp(s, "ERRO") == 0) {
        s_show_error = !s_show_error;
        printf("ERROR,%s\n", s_show_error ? "ON" : "OFF");
        return;
    }

    /* TRIM -> viés de nível p/ base torta.
     *   TRIM        captura a inclinação constante (do integral) p/ o viés fixo
     *   TRIM SHOW   mostra o viés atual
     *   TRIM CLR    zera o viés
     *   TRIM SAVE   grava no NVS (vale após reiniciar)                        */
    if (strcmp(s, "TRIM SHOW") == 0) { cmd_trim_show(); return; }
    if (strcmp(s, "TRIM CLR") == 0 || strcmp(s, "TRIM CLEAR") == 0) {
        s_trim_x = 0.0f; s_trim_y = 0.0f;
        printf("TRIM,zerado\n");
        return;
    }
    if (strcmp(s, "TRIM SAVE") == 0) { cmd_trim_save(); return; }
    if (strcmp(s, "TRIM") == 0 || strcmp(s, "TRIM SET") == 0) {
        cmd_trim_capture();
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

    /* PIDAUTO -> auto-tune adaptativo (balanceia e melhora pelo tempo na mesa). */
    if (strcmp(s, "PIDAUTO") == 0 || strcmp(s, "AUTOPID") == 0) {
        auto_start();
        return;
    }

    /* ZERO -> adiciona o ponto morto atual à lista e SALVA no NVS (acumula).
     * ZEROCLR -> limpa todos os pontos. Vale após reiniciar.               */
    if (strcmp(s, "ZEROCLR") == 0) {
        touch_clear_baseline();
        cal_baseline_t b; memset(&b, 0, sizeof(b)); b.count = 0;
        cal_store_save_baseline(&b);
        printf("ZERO> todos os pontos mortos limpos (NVS).\n");
        return;
    }
    if (strcmp(s, "ZERO") == 0) {
        int n = touch_add_baseline();
        if (n > 0) {
            cal_baseline_t b; memset(&b, 0, sizeof(b));
            b.count = (int8_t)touch_get_baselines(b.x, b.y, CAL_BASE_MAX);
            cal_store_save_baseline(&b);
            printf("ZERO> ponto morto #%d marcado e salvo (NVS). ZEROCLR para limpar.\n", n);
        } else if (n == -2) {
            printf("ZERO> lista cheia (%d). Use ZEROCLR para recomecar.\n", CAL_BASE_MAX);
        } else {
            printf("ZERO> sem leitura para marcar agora. Tente de novo.\n");
        }
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
     *   PID <kp> <ki> <kd>   — ajusta os três; um campo '-' MANTÉM o atual
     *                          (ex.: "PID - 0.5 0.2" mantém Kp, muda Ki e Kd)
     *   KP <v> | KI <v> | KD <v> — ajusta um por vez
     *   ?                    — consulta ganhos e sinais
     *   SX <v> | SY <v>      — sinal do controle por eixo (+1/-1)
     */
    else if (strncmp(s, "PID ", 4) == 0) {
        char t1[24], t2[24], t3[24];
        if (sscanf(s + 4, "%23s %23s %23s", t1, t2, t3) == 3) {
            float kp = s_pid_x.kp, ki = s_pid_x.ki, kd = s_pid_x.kd;
            if (strcmp(t1, "-") != 0) kp = strtof(t1, NULL);   /* '-' = mantém */
            if (strcmp(t2, "-") != 0) ki = strtof(t2, NULL);
            if (strcmp(t3, "-") != 0) kd = strtof(t3, NULL);
            cmd_set_all_gains(kp, ki, kd);
            cmd_print_gains();
        } else {
            printf("ERR,uso: PID <kp> <ki> <kd>   ('-' mantem o atual)\n");
        }
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

/* Versão usual do balanço: inclina pela cinemática (em GEO_HZ) e soma o offset
 * de repouso s_level_steps (definido pela calibração STEPPER em passos). Assim a
 * mesa balança em torno do MEIO físico do curso, sem depender da geometria. */
static void apply_tilt(double nx, double ny, double thoff[3])
{
    long pos[STEPPER_COUNT];
    for (int i = 0; i < STEPPER_COUNT; i++) {
        double theta = machine_theta(&s_machine, i, GEO_HZ, nx, ny);
        if (!isfinite(theta)) theta = s_ang_orig;
        thoff[i] = theta;
        pos[i] = s_level_steps + lround((s_ang_orig - theta) * ANG_TO_STEP);
    }
    steppers_move_to_all(pos);
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

    /* Baselines do ZERO (pontos mortos do painel), se salvos */
    cal_baseline_t saved_base;
    if (cal_store_load_baseline(&saved_base) && saved_base.count > 0) {
        touch_set_baselines(saved_base.count, saved_base.x, saved_base.y);
    }

    /* Viés de nível (TRIM) para a base torta, se salvo */
    cal_trim_t saved_trim;
    if (cal_store_load_trim(&saved_trim)) {
        s_trim_x = saved_trim.nx;
        s_trim_y = saved_trim.ny;
        ESP_LOGI(TAG, "Trim ativo: nx=%.4f ny=%.4f rad", (double)s_trim_x, (double)s_trim_y);
    }

    steppers_init();

    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_set_max_speed(i, SPEED_HOME);
        stepper_set_acceleration(i, ACCEL_HOME);
    }
    steppers_enable(true);
    vTaskDelay(pdMS_TO_TICKS(200));

    double thoff[3];

    /* Carrega o curso calibrado (STEPPER, em passos) do NVS, se existir. */
    cal_steplim_t lim;
    bool have_lim = cal_store_load_steplim(&lim);
    if (have_lim && lim.step_min < lim.step_max) {
        s_step_min = lim.step_min; s_step_max = lim.step_max;
        s_step_min_set = s_step_max_set = true;
        s_level_steps = (s_step_min + s_step_max) / 2;

        /* Startup calibrado: SOBE -> DESCE -> para no MEIO (em passos). */
        ESP_LOGI(TAG, "Startup: curso %ld..%ld passos, meio=%ld",
                 (long)s_step_min, (long)s_step_max, (long)s_level_steps);
        long p[STEPPER_COUNT];
        p[0]=p[1]=p[2]= s_step_max;   steppers_move_to_all(p);
        steppers_run_to_position_blocking(); vTaskDelay(pdMS_TO_TICKS(350));
        p[0]=p[1]=p[2]= s_step_min;   steppers_move_to_all(p);
        steppers_run_to_position_blocking(); vTaskDelay(pdMS_TO_TICKS(350));
        p[0]=p[1]=p[2]= s_level_steps; steppers_move_to_all(p);
        steppers_run_to_position_blocking(); vTaskDelay(pdMS_TO_TICKS(300));
    } else {
        /* Sem calibração de curso: homing antigo (sobe pouco e nivela). */
        s_level_steps = 0;
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
        apply_tilt(0.0, 0.0, thoff);   /* nivela em s_level_steps=0 */
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
    cmd_print_help();
    printf("# Pronto. Digite HELP para a lista, SHOW para ver a tela.\n");
    printf("# ================================================================\n");

    xTaskCreate(cmd_task, "pid_cmd", 3072, NULL, 5, NULL);

    cmd_print_gains();
    cmd_print_signs();
    cmd_print_cal();

    bool detected = false;          /* estado ACTIVE (bola sob controle) */
    int  presence = 0, absence = 0; /* contadores de debounce */
    double last_nx = 0.0, last_ny = 0.0;  /* última inclinação (segura queda curta) */
    unsigned loopn = 0;             /* contador p/ throttle da telemetria */
    TickType_t last_wake  = xTaskGetTickCount();
    TickType_t loop_start = last_wake;   /* início do laço (p/ carência) */
    bool grace_done = false;

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
            /* Calibração de curso: move os 3 motores juntos (nível) em PASSOS. */
            long t[STEPPER_COUNT] = { s_jog_steps, s_jog_steps, s_jog_steps };
            steppers_move_to_all(t);
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
            } else if (!grace_done && (now - loop_start) < pdMS_TO_TICKS(STARTUP_GRACE_MS)) {
                /* Carência: mesa NIVELADA enquanto a leitura estabiliza após o
                 * homing. Ignora qualquer "toque" neste período. */
                detected = false;
                presence = 0; absence = 0;
            } else {
                if (!grace_done) {
                    grace_done = true;
                    printf("STATE,PRONTO (aguardando a bola)\n");
                }
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
                    if (s_pidauto) auto_trial_begin();   /* começa a cronometrar */
                } else if (detected && absence >= PRESENCE_OFF) {
                    detected = false;
                    pid_reset(&s_pid_x);
                    pid_reset(&s_pid_y);
                    printf("STATE,IDLE (sem bola)\n");
                    if (s_pidauto) auto_trial_end(true); /* bola caiu -> pontua e ajusta */
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
                    /* Auto-tune: acumula erro (após acomodação) e fecha por tempo. */
                    if (s_pidauto) {
                        if ((now - s_auto_tstart) > pdMS_TO_TICKS((int)(AUTO_SETTLE_S * 1000))) {
                            float ex = p.x_mm - SETPOINT_X_MM, ey = p.y_mm - SETPOINT_Y_MM;
                            s_auto_errsum += sqrtf(ex * ex + ey * ey);
                            s_auto_errn++;
                        }
                        if ((now - s_auto_tstart) > pdMS_TO_TICKS((int)(AUTO_TRIAL_MAX_S * 1000))) {
                            auto_trial_end(false);   /* sucesso (ficou o tempo todo) */
                            auto_trial_begin();      /* segue testando (bola ainda na mesa) */
                        }
                    }
                }
                /* se !detected: nx=ny=0 -> IDLE (mesa nivelada na posicao padrao) */

                /* Viés de nível (TRIM): compensa a base torta como feedforward
                 * constante. Some na saída em controle E em IDLE, assim a mesa
                 * espera a bola já nivelada e o integral não reaprende isso. */
                nx += s_trim_x;
                ny += s_trim_y;
            }
            apply_tilt(nx, ny, thoff);
        }

        if (s_stepper) {
            /* Status throttled (~3 Hz) da calibração de curso. */
            TickType_t tn = xTaskGetTickCount();
            if (tn - s_cal_live_last >= pdMS_TO_TICKS(300)) {
                s_cal_live_last = tn;
                printf("STEPPER,passos=%ld\n", s_jog_steps);
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

        /* Comando ERROR: erro (distância do centro) a cada 1 s, em qualquer modo. */
        if (s_show_error) {
            TickType_t tn = xTaskGetTickCount();
            if (tn - s_err_last >= pdMS_TO_TICKS(1000)) {
                s_err_last = tn;
                if (touched) {
                    float ex = p.x_mm - SETPOINT_X_MM;
                    float ey = p.y_mm - SETPOINT_Y_MM;
                    float d  = sqrtf(ex * ex + ey * ey);
                    printf("ERROR,ex=%+.1f,ey=%+.1f,dist=%.1fmm\n",
                           (double)ex, (double)ey, (double)d);
                } else {
                    printf("ERROR,sem_bola\n");
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}
