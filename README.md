# Ball Balancer 3RPS — ESP32-S3

Mesa equilibrista controlada por **PID duplo (X e Y)** com **cinemática inversa 3RPS**.
A bola é detectada por uma tela resistiva de 4 fios; três motores NEMA 17 inclinam a
plataforma para manter a bola no centro.

> **Toda a teoria de controle** (modelo da planta, função de transferência, dedução
> dos ganhos `Kp/Ki/Kd`, discretização, anti-windup) está em **[TEORIA.md](TEORIA.md)**.
> Este README cobre apenas **montagem, build e operação**.

---

## 1. Visão geral

```
            bola (tela resistiva)
                  │  x_mm, y_mm  (100 Hz)
                  ▼
     ┌──────────────────────────────┐
     │  PID_X(x_mm) → nx             │   firmware ESP32-S3
     │  PID_Y(y_mm) → ny             │   (laço de 100 Hz)
     │  + viés de nível (TRIM)       │
     │  cinemática 3RPS → θA,θB,θC   │
     │  steppers (perfil trapezoidal)│
     └──────────────────────────────┘
                  │  STEP/DIR
                  ▼
        3× NEMA 17 inclinam a mesa
```

O firmware roda **fechado e autônomo** no ESP32. As ferramentas do PC servem para
**ver, calibrar e ajustar ao vivo** — nada de rebuild para mudar ganhos ou calibração.

---

## 2. Hardware

| Componente | Detalhe |
|---|---|
| Microcontrolador | ESP32-S3 DevKitC-1 |
| Sensor de posição | Tela resistiva 4 fios — VSDISPLAY VS084TP-A1 (8,4 pol) |
| Dimensões da tela | 187 mm × 141 mm (largura × altura) |
| Atuadores | 3× NEMA 17 + drivers STEP/DIR (arranjo 3RPS) |
| Drivers de passo | A4988 / DRV8825 / TMC2208 (interface STEP/DIR idêntica) |

### Pinos — Tela resistiva

Cabo flat com o conector voltado para a **direita** (olhando a superfície da mesa).

| Pino FPC | Sinal | GPIO | Canal ADC |
|---|---|---|---|
| 1 | Y+ | 4 | ADC1_CH3 (sense de X) |
| 2 | X- | 5 | ADC1_CH4 (sense de Y) |
| 3 | Y- | 8 | — (saída digital) |
| 4 | X+ | 9 | Só saída (ADC não funciona neste módulo) |

### Pinos — Drivers de passo

| Motor | STEP | DIR |
|---|---|---|
| A | 15 | 16 |
| B | 17 | 18 |
| C | 40 | 41 |
| ENABLE (comum, ativo LOW) | 10 | — |

### Orientação física

```
         [cabo FPC aqui]
   TL ──────────────── TR
   │                    │
   │       MESA         │
   │     (superfície)   │
   BL ──────────────── BR
```

Convenção de coordenadas após calibração:
- `x_mm = 0` → borda esquerda · `x_mm = 187` → borda direita
- `y_mm = 0` → borda superior · `y_mm = 141` → borda inferior

---

## 3. Estrutura do projeto

```
PIDcV6/
├── README.md              Este arquivo (montagem + operação)
├── SERIAL_COMMANDS.md     Referência completa dos comandos seriais
├── bb_tool.bat/.ps1       Menu de build / flash / monitor (ESP-IDF)
├── TEORIA.md              Teoria de controle digital (PID, FT, ganhos)
├── CMakeLists.txt         Projeto ESP-IDF (target esp32s3)
├── sdkconfig.defaults     Defaults do SDK (sdkconfig é gerado no build)
├── main/                  Firmware C (ESP-IDF)
│   ├── main.c             Entry point + modo MAPPING/CONTROL
│   ├── control.c/.h       Laço 100 Hz + parser serial + TRIM + DANCE/CIRC
│   ├── pid.c/.h           PID por eixo (deriv. filtrada + anti-windup)
│   ├── kinematics.c/.h    Cinemática inversa 3RPS
│   ├── steppers.c/.h      Driver STEP/DIR (perfil trapezoidal 10 kHz)
│   ├── touch_screen.c/.h  Leitura resistiva 4 fios + calibração runtime
│   └── cal_store.c/.h     Persistência NVS (PID, calibração, curso, ZERO, TRIM)
├── tools/                 Ferramentas Python (PC)
│   ├── calibrate.py/.bat  FERRAMENTA PRINCIPAL: calibração da tela, bola ao
│   │                      vivo, orientação (gravar) e ajuste de PID em tempo real
│   ├── PIDSimba.py/.bat   SIMULAÇÃO física visual (motores + bola), precisa pygame
│   ├── sim_pid.py         VALIDADOR offline do PID (headless, só stdlib)
│   ├── config.py          Porta COM, dimensões e orientação dos eixos
│   └── requirements.txt   pygame, pyserial
└── docs/
    └── referencia/        Código Arduino original do Aaed Musa (.ino)
```

---

## 4. As ferramentas

As aplicações Python leem porta/dimensões de `tools/config.py` e dependem de
`pygame` + `pyserial` (`pip install -r tools/requirements.txt`).

> **Build, flash e monitor** não têm GUI própria — use o **`bb_tool`** (menu
> interativo na raiz do projeto, ver §5) ou o `idf.py` direto. Depois da primeira
> gravação, **calibração, ganhos, sinais e o viés de nível (TRIM) são ajustados ao
> vivo pelo terminal serial (ver `SERIAL_COMMANDS.md`), sem nunca mais rebuildar**.

### 🔧 `bb_tool` — Build / Flash / Monitor (raiz do projeto)

Menu interativo (`bb_tool.bat`) que carrega o ambiente ESP-IDF e oferece build,
*fullclean* + *set-target* `esp32s3`, flash, monitor e flash+monitor. Porta padrão
**COM8** (ajuste no menu). É o caminho recomendado para gravar e abrir o monitor.

### 🛠️ `calibrate.py` — Ferramenta principal de operação

Abra por `tools/calibrate.bat` (ou `python tools/calibrate.py COM5`).
É o app do dia a dia e cobre os quatro recursos centrais. Telas:

| Tecla / Estado | Para quê |
|---|---|
| `ENTER` na splash | Inicia a **calibração** guiada (coleta 80 amostras em cada um dos 4 cantos) |
| `L` na splash | Pula direto ao LIVE usando a calibração de `config.py` |
| Em **CAL_RESULTS** | `S` salva no header (.h); **`N` envia ao NVS do ESP32 (sem rebuild)**; `ENTER` vai ao LIVE |
| **LIVE** | **Bola ao vivo** em mm + trilha. `T`/`X`/`Y` ajustam a **orientação**; **`G` grava** (config + header) |
| **PID** (tecla `P` no LIVE) | **Ajuste de KP/KI/KD em tempo real** contra o firmware: `Q/A`,`W/S`,`E/D` mudam os ganhos, `Tab` o passo, **`ESPAÇO` salva no NVS** |
| **BALANCE** (tecla `B` no LIVE) | Simulação isométrica dos 3 motores reagindo à bola real, com os ganhos atuais |

> O modo **PID** requer o firmware em **modo CONTROL** (padrão). Ao entrar, ele
> envia `?` ao ESP para ler os ganhos em vigor (`GAINS`) e sincronizar o painel —
> a partir daí cada ajuste é aplicado na hora e `ESPAÇO` persiste no NVS.

### 🧮 `sim_pid.py` — Validador do PID offline (sem hardware, sem pygame)

Para **testar se os ganhos PID funcionam ANTES de ir ao firmware**. Usa só a
biblioteca padrão do Python (roda em qualquer máquina, sem instalar nada). Porta
fiel do algoritmo de `pid.c` contra o modelo de planta da [TEORIA.md](TEORIA.md):
roda a resposta no laço fechado e imprime overshoot, tempo de resposta,
acomodação, erro residual e um gráfico ASCII.

```
python tools/sim_pid.py                          # compara firmware x ganhos recomendados
python tools/sim_pid.py 1.08e-3 6.83e-4 5.4e-4   # testa os ganhos do TCC (ou os SEUS)
python tools/sim_pid.py 8e-4 2e-5 1.2e-2 --x0 70 --dist 200 --t 15
```

`--x0` deslocamento inicial (mm) · `--dist` chute de velocidade no meio (mm/s) ·
`--t` duração (s). O resultado bate com a previsão analítica (`ωn`, `ζ`) impressa
junto — é a forma rápida de descobrir bons KP/KI/KD para depois enviá-los ao ESP
pela tela PID do `calibrate.py`.

> **Modelo:** plano linearizado de eixo único (`x'' = −A·u`, `A = (5/7)g`). Valida
> a *matemática do controlador*. A dinâmica não-linear completa (cinemática 3RPS,
> motores reais) está no `PIDSimba.py`.

### 🧪 `PIDSimba.py` — Simulação física dedicada (visual)

Abra por `tools/PIDSimba.bat`. Foco em **demonstração e ensino**. A dinâmica segue
o modelo do TCC de **Cordeiro (UNESP, 2022)** — **controle em cascata** (PID de
posição → inclinação → 3 motores via cinemática inversa) — ver [TEORIA.md](TEORIA.md) §5.

- **Modo SIM**: bola virtual rolando no plano sob gravidade real; a mesa se inclina
  para **trazer a bola de volta ao centro** e os 3 motores realizam a inclinação.
  Converge dos 4 cantos sem hardware. Ganhos projetados por alocação de polos
  (TCC) e amortecidos ("PID melhorado", ζ≈1,15) — ver [TEORIA.md](TEORIA.md) §5.
- **Modo REAL**: lê a telemetria do ESP (`POS`/`CTRL`) e **espelha os ângulos reais**
  dos 3 motores que o firmware comandou.

```
python tools/PIDSimba.py            # usa config.py
python tools/PIDSimba.py COM5       # porta explícita
python tools/PIDSimba.py --sim      # abre direto na simulação física
```

**Como ver o PID trabalhando (modo SIM):**

| Ação | Tecla / mouse |
|---|---|
| **Mover a bola** (e ver o PID trazê-la de volta) | **arraste com o mouse**, solte |
| Empurrão rápido na bola | `P` |
| Mover o alvo (a bola persegue) | `WASD` |
| Vista de topo (arrasto mais intuitivo) | `V` |
| Escolher ganho Kp/Ki/Kd | `1` / `2` / `3` |
| Aumentar/diminuir o ganho escolhido (×1,12) | `↑` / `↓`  ·  `0` reseta |

> Por padrão a bola vai ao centro e fica parada — é o PID funcionando. Para ver o
> **comportamento dinâmico**, arraste a bola para um canto e solte: ela oscila/volta
> conforme os ganhos. Diminua o `Kd` (tecla `3` + `↓`) para vê-la oscilar mais.

---

## 5. Fluxo de trabalho

### Primeira configuração (hardware novo)

1. Edite a porta COM em `tools/config.py` (e no `bb_tool` se não for COM8).
2. **Grave o firmware** (uma vez) pelo `bb_tool` (flash) ou `idf.py`:
   ```
   idf.py build
   idf.py -p COM8 flash monitor
   ```
   Confirme no monitor: `Ball Balancer — Control Mode @ 100 Hz`. O console é o
   **USB-Serial-JTAG** (conector USB nativo do ESP32-S3).
3. **Calibre a tela** pelo terminal: `CAL` → coloque a bola nos **4 cantos**
   (o centro é derivado deles) → `SALVAR`. (Alternativa visual: `calibrate.bat`.)
4. **Calibre o curso dos motores:** `STEPPER` → `SOBE/DESCE`, `MIN`, `MAX` → `SALVAR`.
5. **Ajuste o PID** ao vivo: `ERROR` (monitor de erro), `PID - - 0.012` (damping),
   suba `Ki` até centralizar, `PID SAVE`. Depois `TRIM` + `TRIM SAVE` para a base torta.

### Boots seguintes

O firmware **carrega calibração e ganhos do NVS automaticamente**. Não precisa
recalibrar nem rebuildar — só ligar.

| Situação | Ação |
|---|---|
| Mudou posição/orientação da tela | `calibrate.py`: recalibrar → `N`, e/ou `T`/`X`/`Y` → `G` |
| Quer mexer no PID | `calibrate.py`: tecla `P` → ajustar → `ESPAÇO` |
| Mesa empurra a bola para longe (um eixo) | Inverter aquele eixo ao vivo: `SX +1` ou `SY +1` (default já é −1) |
| Apagou a flash (`idf.py erase-flash`) | Refazer calibração + PID |

---

## 6. Ajuste ao vivo (sem rebuild)

Tudo abaixo é aplicado **na hora** e pode ser persistido no NVS. É o que torna
o tuning prático — você não recompila para testar um ganho.

### Protocolo serial (USB-Serial-JTAG, 115200 baud)

A referência completa está em **[SERIAL_COMMANDS.md](SERIAL_COMMANDS.md)**. Resumo:

**Saída do firmware (telemetria com `SHOW`, 100 Hz):**

```
POS,<x_raw>,<y_raw>,<x_mm>,<y_mm>            bola detectada
NOTOUCH,<x_raw>,<y_raw>                       sem contato
CTRL,<x>,<y>,<setx>,<sety>,<nx>,<ny>,<thA>,<thB>,<thC>
ERROR,ex=..,ey=..,dist=..mm                   com ERROR ligado (1 Hz)
STATE,<PRONTO|ACTIVE|IDLE>                    transições da bola
GAINS,<kp>,<ki>,<kd>   SIGNS,<sx>,<sy>   TRIM,<nx>,<ny>
CAL,<xmin>,<xmax>,<ymin>,<ymax>,<fx>,<fy>,<sw>
SAVED,<pid|touch|trim>                         confirmação de NVS salvo
```

**Comandos principais:**

| Comando | Efeito |
|---|---|
| `HELP` · `?` | Lista comandos · estado atual (GAINS/SIGNS/CAL) |
| `SHOW` / `HIDDEN` | Liga / desliga telemetria |
| `ERROR` | Imprime o erro (dist. do centro) a cada 1 s |
| `PID <kp> <ki> <kd>` | Ajusta os três (`-` mantém o atual); `PID SAVE` grava |
| `KP/KI/KD <v>` · `SX/SY <v>` | Um ganho por vez · sinal do controle por eixo |
| `TRIM` · `TRIM SAVE` | Aprende/grava o viés de nível (base torta) |
| `CAL` | Assistente de calibração da tela (4 cantos; centro derivado) |
| `STEPPER` | Calibra o curso dos motores (MIN/MAX/MEIO) |
| `PIDAUTO [kp ki kd]` | Ajuste fino dos ganhos + auto-TRIM (parte dos ganhos atuais) |
| `ELIPSE` · `DANCE` | Varredura elíptica (sem bola) · dancinha de exibição |
| `CIRC` | Gira a bola num círculo suave no centro (com bola; `PARAR` sai) |
| `ZERO` / `ZEROCLR` | Marca / limpa pontos mortos da tela |

> Tudo pode ser feito por qualquer monitor serial (ou pelo `bb_tool` → monitor).
> O `calibrate.py` também monta os comandos de calibração/PID por GUI.

---

## 7. Modos de firmware

| Macro `BB_MODE` | Comportamento |
|---|---|
| `BB_MODE_CONTROL` (padrão) | Laço PID 100 Hz + cinemática + motores |
| `BB_MODE_MAPPING` | Só lê o touch e emite telemetria — sem motores |

Compilar em modo mapeamento (útil só para depurar a tela):
`idf.py build -DBB_MODE=BB_MODE_MAPPING`

---

## 8. Parâmetros de geometria

Definidos em `main/control.c`. **São placeholders do projeto de referência —
medir no rig real e substituir.**

| Constante | Valor atual | Significado |
|---|---|---|
| `GEO_D` | 50,8 mm | Raio da base (pivôs inferiores) |
| `GEO_E` | 79,4 mm | Raio da plataforma (juntas esféricas) |
| `GEO_F` | 44,45 mm | Comprimento do braço do motor |
| `GEO_G` | 93,2 mm | Comprimento da biela |
| `GEO_HZ` | 108,0 mm | Altura neutra da plataforma |
| `MICROSTEPS` | 16 | Microstepping dos drivers (DIP switches) |

A relação dos steppers (`SPEED_BALANCE`, `ACCEL_BALANCE`, etc.) e a justificativa
do perfil trapezoidal estão em [TEORIA.md](TEORIA.md).

---

## 9. Configuração atual e pendências

**Estado validado em operação** (envie `?` para reler):

```text
GAINS,0.0006,3e-05,0.0001      # Kp=6e-4  Ki=3e-5  Kd=1e-4
SIGNS,-1,-1                     # realimentação negativa (confirmado no rig)
CAL,513,3297,355,3719,1,1,1    # calibração da tela (4 cantos)
```

| Item | Status |
|---|---|
| Sinais `SX`/`SY` (realimentação negativa) | ✅ Confirmado no rig (−1/−1) |
| Calibração da tela (4 cantos) | ✅ Feita e salva no NVS |
| Curso dos motores (`STEPPER`) | ✅ Calibrado (repouso no meio do curso) |
| Detecção de toque (digital) + rejeição de outliers | ✅ Estável nos cantos |
| Equilíbrio + `TRIM` para base torta | ✅ Operacional |
| Medir geometria real (`GEO_D/E/F/G/HZ`) | Pendente (placeholders) |
| Confirmar `MICROSTEPS` (DRV8825 em 1/16) | ✅ Verificado (M0/M1/M2 = L/L/H) |
| Tuning fino de KP/KI/KD | Refinamento contínuo via `PID`/`PIDAUTO` |

---

## Créditos

Cinemática inversa 3RPS e arquitetura original baseadas no projeto
[Ball Balancer do Aaed Musa](https://www.instructables.com/Ball-Balancer/).
O código Arduino original está em [docs/referencia/](docs/referencia/).
