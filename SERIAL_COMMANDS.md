# Menu Interativo — Protocolo Serial ESP32-S3 Ball Balancer

**Porta:** `COM*` (ajuste no seu SO)  
**Baud rate:** `115200`  
**Formato:** linhas texto, terminadas com `\n` ou `\r\n`

---

## Estados da Aplicação

### 1. **Configuração (Boot)**
Após ligar o ESP32, a serial exibe:
- Configuração de motores (pinos, inversão)
- Ganhos PID atuais (salvos no NVS ou defaults)
- Sinais dos eixos (SX, SY)
- Calibração de touch
- Mensagem: `Aguardando START para iniciar telemetria...`

**Neste estado:** você pode enviar comandos (PID, CAL, etc.). Nenhuma telemetria sai automaticamente.

### 2. **Telemetria Ativa (START)**
Envie `START` → a aplicação começa a transmitir posição/controle em tempo real.

**Neste estado:** a cada ciclo (50 Hz) sai uma ou duas linhas:
- `POS,x_raw,y_raw,x_mm,y_mm` ou `NOTOUCH,x_raw,y_raw`
- `CTRL,x_mm,y_mm,setpoint_x,setpoint_y,tilt_x,tilt_y,theta_a,theta_b,theta_c`

Você ainda pode enviar comandos (eles são processados, mas não interrompem a telemetria).

### 3. **Telemetria Desativada (STOP)**
Envie `STOP` → a aplicação para de transmitir dados.

Volta ao estado 1: limpo, silencioso, pronto para novos ajustes.

---

## Comandos Disponíveis

### **Telemetria**

| Comando | Resposta | Efeito |
|---------|----------|--------|
| `START` | `TELEM,ON` | Habilita saída de telemetria (50 Hz) |
| `STOP` | `TELEM,OFF` | Desabilita telemetria |

### **PID — Ganhos**

| Comando | Exemplo | Resposta | Efeito |
|---------|---------|----------|--------|
| `PID <kp> <ki> <kd>` | `PID 0.001 0.00001 0.015` | `GAINS,0.001,0.00001,0.015` | Ajusta os 3 ganhos de uma vez (ambos eixos) |
| `KP <valor>` | `KP 0.001` | `GAINS,...` | Ajusta só KP (proporcional) |
| `KI <valor>` | `KI 0.00001` | `GAINS,...` | Ajusta só KI (integral) |
| `KD <valor>` | `KD 0.015` | `GAINS,...` | Ajusta só KD (derivativo) |
| `PID SAVE` | — | `SAVED,pid` + `GAINS,...` | Salva no NVS flash (persiste após reboot) |

> **Nota:** ajustes são imediatos em ambos os eixos (X e Y). Use `SX`/`SY` para inverter sinais individuais.

### **Sinais dos Eixos**

| Comando | Exemplo | Resposta | Efeito |
|---------|---------|----------|--------|
| `SX <valor>` | `SX -1` ou `SX 1` | `SIGNS,...` | Inverte (ou normaliza) o sinal do eixo X |
| `SY <valor>` | `SY -1` ou `SY 1` | `SIGNS,...` | Inverte o eixo Y |

> **Quando usar:** se ao soltar a bola levemente fora do centro, a mesa **empurra a bola pra longe** (realimentação positiva), inverta com `SX -1` ou `SY -1`. Teste e ajuste.

### **Calibração do Touch**

#### Captura de Cantos

```
CAL TL      # toque o canto superior-esquerdo (Top-Left)
> CORNER,TL,x_raw,y_raw

CAL TR      # toque o canto superior-direito
> CORNER,TR,x_raw,y_raw

CAL BL      # toque o canto inferior-esquerdo
> CORNER,BL,x_raw,y_raw

CAL BR      # toque o canto inferior-direito (onde sai o cabo FPC)
> CORNER,BR,x_raw,y_raw
```

#### Aplicar Calibração

```
CAL APPLY   # calcula mapping automático dos 4 cantos
> CAL,xmin,xmax,ymin,ymax,flip_x,flip_y,swap_xy
```

Nesta etapa, a aplicação **auto-detecta** se os eixos estão trocados (`swap_xy`) ou invertidos (`flip_x`, `flip_y`).

#### Salvar

```
CAL SAVE    # persiste no NVS (usada no próximo boot)
> SAVED,touch
> CAL,...
```

#### Resetar / Consultar

| Comando | Resposta | Efeito |
|---------|----------|--------|
| `CAL SHOW` | `CAL,xmin,...` | Exibe calibração atual |
| `CAL RESET` | `CAL,665,2981,...` | Restaura defaults de compilação |
| `SETCAL <xmin> <xmax> <ymin> <ymax> <fx> <fy> <sw>` | `CAL,...` | Aplica calibração direta (sem capturar cantos) |

### **Status Geral**

| Comando | Resposta |
|---------|----------|
| `?` | Exibe `GAINS`, `SIGNS` e `CAL` atuais |

---

## Fluxo Típico de Uso

### **1️⃣ Primeira Energização**

```
[Ligar ESP32]
# Ball Balancer — ESP32-S3 | 3RPS | 50 Hz
# Motor inversão (steppers.h): MOT_A=0 MOT_B=0 MOT_C=0
# ...
# Aguardando START para iniciar telemetria...
GAINS,0.0008,0.00002,0.012
SIGNS,+1,+1
CAL,665,2981,554,3273,1,0,1
```

✅ Se **homing subiu ~15 mm e nivelou**: continua.  
❌ Se **homing desceu (colidiu com base)**: algum motor está invertido.

```
# Editar steppers.h
#define MOTOR_X_INVERT 1  # onde X = A, B ou C (o que desceu)

# Recompilar e reflashtear
idf.py build
idf.py flash
```

---

### **2️⃣ Calibração do Touch**

Prepare a mesa nivelada, com bola leve à mão.

```
CAL TL
[Toque o canto superior-esquerdo]
> CORNER,TL,xxx,yyy

CAL TR
[Toque o canto superior-direito]
> CORNER,TR,xxx,yyy

CAL BL
[Toque o canto inferior-esquerdo]
> CORNER,BL,xxx,yyy

CAL BR
[Toque o canto inferior-direito — perto do cabo FPC]
> CORNER,BR,xxx,yyy

CAL APPLY
> CAL,xmin,xmax,ymin,ymax,flip_x,flip_y,swap_xy

[Verifique:]
?
> GAINS,...
> SIGNS,...
> CAL,665,2981,554,3273,1,0,1  ← deve ter valores coerentes

CAL SAVE
> SAVED,touch
```

---

### **3️⃣ Teste Inicial — Sinais**

Ganho baixo, verificação de direção.

```
KP 0.0001
KI 0
KD 0

START
# Agora a telemetria está ativa

[Solte a bola suavemente fora do centro]
```

**Verifique:**
- ✅ A mesa **inclina na direção da bola** (realimentação negativa) → sinais corretos
- ❌ A mesa **empurra a bola pra longe** (realimentação positiva) → inverta o sinal

```
# Se realimentação positiva em X:
SX -1
# Se em Y:
SY -1

STOP
```

---

### **4️⃣ Sintonia do PID**

Agora com sinais verificados, suba os ganhos.

```
# Comece com KP baixo, sem I e D
KP 0.0005
KI 0
KD 0

START

[Solte a bola levemente deslocada]
```

**Observe a resposta:**
- Se não vai pro centro → aumente KP
- Se oscila muito → aumente KD (derivativo amortece)
- Se fica com erro residual → pequeno KI para eliminar drift

**Exemplo iterativo:**
```
STOP
KP 0.0008
START
[teste...]

STOP
KD 0.01
START
[teste...]

STOP
KI 0.00002
KP 0.0008
KD 0.012
PID SAVE
START
```

**Dica:** use `LOOP_DT = 0.02 s` na sua cabeça. Se KP=0.0008, em erro de 100 mm a saída é `0.0008 × 100 = 0.08` (tilt), o que inclina uns ~4-5 graus.

---

## Estrutura de Saída de Telemetria

### POS (toque detectado)
```
POS,<x_raw>,<y_raw>,<x_mm>,<y_mm>
```
- `x_raw`, `y_raw`: valores ADC brutos (0-4095)
- `x_mm`, `y_mm`: posição calibrada em milímetros [0..187] × [0..141]

### NOTOUCH
```
NOTOUCH,<x_raw>,<y_raw>
```
Nenhum toque detectado (ADC abaixo do threshold ~300).

### CTRL (controle)
```
CTRL,<x_mm>,<y_mm>,<setpoint_x>,<setpoint_y>,<tilt_x>,<tilt_y>,<theta_a>,<theta_b>,<theta_c>
```
- `x_mm`, `y_mm`: posição atual (ou -1 se não há toque)
- `setpoint_x`, `setpoint_y`: alvo (93.5, 70.5 mm)
- `tilt_x`, `tilt_y`: saída do PID (inclinação desejada, clamped a ±0.25)
- `theta_a`, `theta_b`, `theta_c`: ângulos dos motores (graus)

---

## Erros Comuns

| Problema | Causa | Solução |
|----------|-------|--------|
| Homing desci para a base | Motor invertido | `MOTOR_X_INVERT 1` em `steppers.h` |
| Telemetria não sai | Esqueceu `START` | Envie `START` |
| Bola sai voando | Sinal errado | `SX -1` ou `SY -1` |
| Não atinge o centro | KP muito baixo | Aumente `KP` |
| Oscila muito | KD muito baixo | Aumente `KD` |
| Calibração errada | Cantos não capturados direito | `CAL RESET` e refaça `CAL TL/TR/BL/BR` |
| Comando não funciona | Typo ou case-sensitive | Envie em MAIÚSCULAS ou verifique com `?` |

---

## Exemplo Completo de Sessão

```
[ESP32 ligado]
# ... mensagens de boot ...
GAINS,0.0008,0.00002,0.012
SIGNS,+1,+1
CAL,665,2981,554,3273,1,0,1

[Calibração rápida]
CAL TL
> CORNER,TL,700,600
CAL TR
> CORNER,TR,2900,600
CAL BL
> CORNER,BL,700,3200
CAL BR
> CORNER,BR,2900,3200
CAL APPLY
> CAL,700,2900,600,3200,0,0,0
CAL SAVE
> SAVED,touch

[Teste com ganho baixo]
KP 0.0001
KI 0
KD 0
START
> TELEM,ON
POS,1500,1700,93.4,70.2
CTRL,93.4,70.2,93.5,70.5,-0.0001,0.0003,108.00,108.00,108.00
POS,1502,1698,93.6,70.1
CTRL,93.6,70.1,93.5,70.5,0.0001,0.0001,108.00,108.00,108.00
[bola se move pro centro — sinal correto ✓]

STOP
> TELEM,OFF

[Sintonia]
KP 0.0008
KD 0.012
PID SAVE
> SAVED,pid
> GAINS,0.0008,0,0.012

START
> TELEM,ON
[teste final com ganho completo...]
```

---

## Referência Rápida

```
START                          # Habilita telemetria
STOP                           # Desabilita telemetria

KP <v>                         # Define ganho proporcional
KI <v>                         # Define ganho integral
KD <v>                         # Define ganho derivativo
PID <kp> <ki> <kd>            # Define os 3 de uma vez
PID SAVE                       # Salva no NVS

SX <-1|1>                      # Inverte/normaliza eixo X
SY <-1|1>                      # Inverte/normaliza eixo Y

CAL TL/TR/BL/BR               # Captura cantos
CAL APPLY                      # Aplica mapeamento
CAL SAVE                       # Salva calibração
CAL RESET                      # Restaura defaults
CAL SHOW                       # Exibe calibração atual
SETCAL <7 valores>            # Força calibração

?                              # Exibe estado (GAINS, SIGNS, CAL)
```

---

## Notas

- **Todos os comandos são case-insensitive** (maiúsculas/minúsculas não importam).
- **NVS persistence:** comandos `PID SAVE` e `CAL SAVE` armazenam no flash e são carregados no boot.
- **Telemetria é **não-bloqueante** → comandos chegam mesmo com streaming ativo.
- **Segurança:** homing roda apenas ao boot. Se desligar/ligar, repita a sequência.
