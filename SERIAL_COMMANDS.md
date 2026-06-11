# Menu Interativo — Protocolo Serial ESP32-S3 Ball Balancer

**Console:** USB-Serial-JTAG (o conector USB **nativo** do ESP32-S3, não a UART externa)
**Baud:** 115200 (o USB-CDC ignora o baud, mas configure assim mesmo)
**Formato:** linhas de texto terminadas com `\n` ou `\r\n`
**Comandos são *case-insensitive*** (o firmware converte para maiúsculas).

> A forma mais prática de abrir o monitor é pelo **`bb_tool`** (menu na raiz do
> projeto): opção *monitor* (ou *flash+monitor*) na COM do dispositivo.

---

## Configuração atual em operação (registro)

Estado validado no rig (envie `?` para reler a qualquer momento):

```text
GAINS,0.0008,2e-05,0.012       # Kp=8e-4  Ki=2e-5  Kd=1.2e-2  (defaults do firmware)
SIGNS,-1,-1                     # realimentação negativa nos dois eixos
CAL,665,2981,554,3273,1,1,1    # xmin,xmax,ymin,ymax,flip_x,flip_y,swap_xy
```

O **TRIM** (viés de nível da base torta) é salvo à parte (`TRIM SHOW` para ver).

---

## Visão geral dos estados

1. **Boot** — após o *homing* (sobe → desce → para no meio do curso) a mesa faz
   uma **dancinha** de boas-vindas (ver `DANCE`) e então fica nivelada por ~1,5 s
   (carência), entrando em `STATE,PRONTO`. Sem bola fica em `STATE,IDLE`
   (nivelada, já aplicando o TRIM); com bola, `STATE,ACTIVE`.
2. **Telemetria** — desligada por padrão. `SHOW` (ou `START`) liga; `HIDDEN`
   (ou `STOP`) desliga. Comandos continuam funcionando com a telemetria ligada.
3. **Modos guiados** — `CAL`, `STEPPER`, `PIDAUTO`, `ELIPSE` assumem o terminal
   com seus próprios subcomandos até você sair (`SALVAR`/`CANCELAR`/`PARAR`).
4. **Modos de exibição** — `DANCE` (coreografia, sem bola) e `CIRC` (gira a bola
   no centro, **com** bola). `CIRC` sai com `PARAR`.

---

## Comandos

### Ajuda e estado

| Comando | Efeito |
|---|---|
| `HELP` / `AJUDA` | Lista todos os comandos |
| `?` | Imprime `GAINS`, `SIGNS` e `CAL` atuais |
| `SHOW` / `START` | Liga a telemetria (`POS`/`NOTOUCH` + `CTRL`) |
| `HIDDEN` / `HIDE` / `STOP` | Desliga a telemetria |
| `ERROR` / `ERRO` | Liga/desliga a impressão do **erro** (dist. do centro) a cada 1 s |
| `ERROR OFF` | Desliga o monitor de erro |

### PID — ganhos

| Comando | Exemplo | Efeito |
|---|---|---|
| `PID` | — | Mostra o menu com os ganhos atuais |
| `PID <kp> <ki> <kd>` | `PID 0.0008 2e-5 0.012` | Ajusta os 3 (ambos os eixos) |
| `PID <a> <b> <c>` com `-` | `PID - 0.5 0.2` | **`-` mantém** o ganho atual (aqui mantém Kp) |
| `KP <v>` / `KI <v>` / `KD <v>` | `KD 0.012` | Ajusta um ganho por vez |
| `PID SAVE` | — | Persiste os ganhos no NVS (`SAVED,pid`) |

> Eco após qualquer ajuste: `GAINS,kp,ki,kd`.

### Sinais dos eixos

| Comando | Efeito |
|---|---|
| `SX <v>` / `SY <v>` | Sinal do controle por eixo (`<0` → −1, senão +1). Eco `SIGNS,sx,sy` |

> Use se um eixo **empurrar a bola para longe** (realimentação positiva). O default é −1/−1.

### TRIM — viés de nível (base torta)

Compensa a estrutura torta como *feedforward* constante: a inclinação que o
integral descobriu vira um offset fixo, então a mesa já espera a bola nivelada.

| Comando | Efeito |
|---|---|
| `TRIM` / `TRIM SET` | Captura a inclinação atual do integral para o viés fixo (e zera o integral) |
| `TRIM SHOW` | Mostra o viés atual (`TRIM,nx=..,ny=.. rad (.. graus)`) |
| `TRIM CLR` / `TRIM CLEAR` | Zera o viés |
| `TRIM SAVE` | Persiste no NVS (vale após reiniciar) |

**Uso típico:** com a bola equilibrada e já centralizada (~10–20 s), `TRIM` →
`TRIM SAVE`. A partir daí a bola nasce no centro a cada nova colocação.

### CAL — calibração da tela (assistente guiado, 4 cantos)

`CAL` (ou `CALIBRAR`) inicia o assistente. A mesa fica nivelada (não balança) e
sai um stream `LIVE` enxuto. O **centro é derivado dos 4 cantos** (não é medido,
para não enviesar). Subcomandos durante o assistente:

| Passo / Comando | Efeito |
|---|---|
| `[1..4/4]` + `OK` | Coloque a bola no canto pedido e confirme |
| `CANCELAR` / `ABORT` | Aborta e restaura a calibração anterior |
| `SALVAR` (na confirmação) | Grava no NVS (`SAVED,touch`) |

Sequência: SUP-ESQ → SUP-DIR → INF-DIR → INF-ESQ → (centro calculado) → `SALVAR`.

#### CAL — modo manual (sem assistente)

| Comando | Efeito |
|---|---|
| `CAL TL` / `TR` / `BL` / `BR` | Captura o toque atual como aquele canto |
| `CAL APPLY` | Calcula a calibração dos 4 cantos (auto-detecta swap/flip) |
| `CAL SAVE` / `CAL SHOW` / `CAL RESET` | Persiste / exibe / restaura defaults |
| `SETCAL <xmin> <xmax> <ymin> <ymax> <fx> <fy> <sw>` | Aplica calibração calculada no PC |

### STEPPER — calibração do curso dos motores

`STEPPER` (ou `MOTOR`) inicia o assistente para achar o curso útil (em passos).

| Comando | Efeito |
|---|---|
| `SOBE` / `SUBIR` / `S` · `DESCE` / `DESCER` / `D` | Move os 3 motores juntos (nivelados) |
| `MINIMO` / `MIN` · `MAXIMO` / `MAX` | Marca o ponto baixo / alto do curso |
| `MEIO` | Vai ao meio do curso |
| `SALVAR` | Grava `min/max` no NVS (repouso = meio do curso) |
| `CANCELAR` | Sai sem salvar |

### PIDAUTO — ajuste fino adaptativo (+ auto-TRIM)

`PIDAUTO` é um **ajuste fino**: parte de ganhos que **já equilibram** e refina em
torno deles (não busca do zero). A referência é o **centro da mesa**.

| Comando | Efeito |
|---|---|
| `PIDAUTO` | Parte dos **ganhos atuais** e refina |
| `PIDAUTO <kp> <ki> <kd>` | Parte dos ganhos informados (ex.: `PIDAUTO 0.0008 2e-5 0.012`) |
| `PARAR` | Encerra e aplica o melhor conjunto (depois `PID SAVE` para gravar) |

Como funciona / salvaguardas:

- **Auto-TRIM:** após ~10 s com a bola a < 8 mm do centro, captura o nível da
  base sozinho (equivale ao `TRIM`) e zera o integral — uma vez por sessão.
- **Custo = erro + agitação:** pune a velocidade da bola, não só a distância ao
  centro, então um ajuste nervoso pontua **pior** que a semente.
- **Preso à semente:** cada ganho fica limitado a `semente × [0,6 ; 1,6]`.
- **Queda virtual:** como há fita nas bordas, passar de ~62 mm do centro conta
  como **queda** (a bola "teria caído") e penaliza o ajuste.
- **Aborto precoce:** se a bola passar de ~45 mm durante um teste, aborta na hora
  e reverte para o melhor — evita "empurrar a bola pra fora".

> Log por tentativa: `PIDAUTO> #n ok t=..s err=..mm agit=..mm/s score=.. -> ...`.

### ELIPSE — teste de movimento (sem bola)

`ELIPSE` (ou `ELLIPSE`) faz a mesa varrer uma elipse suave (malha aberta, sem
PID), para testar mecânica/cinemática. `PARAR` sai.

### DANCE — dancinha de exibição (sem bola)

`DANCE` (ou `DANCA`/`DANCAR`) roda a **coreografia de boas-vindas** — a mesma que
toca no boot. Usa o curso calibrado dos motores (`STEPPER`), sempre **preso aos
limites** `min/max`. Sequência: respira → olha em volta (círculo) → "sim" → "não"
→ onda rolante (braços defasados 120°) → giro final → nivela.

> Sem curso calibrado, faz só um aceno gentil em tilt (amplitude pequena).
> Roda na task dos motores, então não conflita com o balanço.

### CIRC — gira a bola no centro (com bola)

`CIRC` (ou `CIRCULO`/`CIRCLE`) é **malha fechada**: o setpoint gira num círculo
pequeno em torno do centro e a **bola segue**, circulando suavemente. Precisa da
bola na mesa. `PARAR` volta a segurar no centro.

- Raio padrão **22 mm**, ~**8 s/volta** (`CIRCLE_R_MM` / `CIRCLE_HZ` em `control.c`).
- Eco: `CIRC,ON (raio=..mm, ~..s/volta; PARAR para sair)` / `CIRC,OFF`.

### ZERO — pontos mortos da tela

| Comando | Efeito |
|---|---|
| `ZERO` | Marca o ponto atual como "preso" (sem bola), **acumula** e salva no NVS |
| `ZEROCLR` | Limpa todos os pontos mortos |

> Paliativo para painéis com contato espúrio fixo: leituras perto de um ponto
> `ZERO` (tolerância ~8 mm) viram `NOTOUCH`.

---

## Saída de telemetria

| Mensagem | Significado |
|---|---|
| `POS,xr,yr,xmm,ymm` | Bola detectada (ADC bruto + posição em mm) |
| `NOTOUCH,xr,yr` | Sem contato |
| `CTRL,x,y,setx,sety,nx,ny,thA,thB,thC` | Estado do laço (pos., setpoint, inclinação, ângulos dos 3 motores) |
| `ERROR,ex=+..,ey=+..,dist=..mm` | Erro a cada 1 s (com `ERROR` ligado); `ERROR,sem_bola` se vazio |
| `STATE,PRONTO\|ACTIVE\|IDLE` | Transições de estado da bola |
| `LIVE,xr,yr,xmm,ymm` | Stream enxuto durante o `CAL` |
| `GAINS,kp,ki,kd` · `SIGNS,sx,sy` · `CAL,...` · `TRIM,...` | Ecos de configuração |
| `SAVED,pid\|touch\|trim` | Confirmação de gravação no NVS |

`setpoint` = centro da tela = **(93,5 ; 70,5) mm** (no modo `CIRC` ele **gira** em
torno do centro). `nx,ny` saturam em **±0,25**.

---

## Fluxo recomendado (resumo)

```text
# 1) Calibrar a tela (uma vez)
CAL                      # 4 cantos -> SALVAR

# 2) Calibrar o curso dos motores (uma vez)
STEPPER                  # SOBE/DESCE, MIN, MAX -> SALVAR

# 3) Ajustar o PID
ERROR                    # liga o monitor de erro
PID - - 0.012            # damping (Kd) é o que estabiliza o duplo integrador
PID - 0.0001 -           # sobe Ki até centralizar (dist -> 0), sem vaguear
PID SAVE

# 4) Compensar a base torta
TRIM                     # com a bola já centralizada
TRIM SAVE
```

---

## Erros comuns

| Sintoma | Causa provável | Solução |
|---|---|---|
| *Homing* desce e bate na base | Algum motor invertido | `MOTOR_*_INVERT` em `steppers.h` |
| Mesa "surta do nada" | `Kd` baixo (sub-amortecido) ou leitura espúria | Suba `Kd`; a rejeição de outliers já filtra glitches |
| Bola para deslocada e só centra devagar | `Ki` fraco p/ base torta | Use `TRIM` (+ `TRIM SAVE`); suba `Ki` |
| Bola "voa" para longe | Sinal errado | `SX -1` ou `SY -1` |
| "Equilibra o nada" num ponto | Contato preso no painel | `ZERO` naquele ponto (e `ZEROCLR` para refazer) |
| Cantos não detectam a bola | (resolvido) detecção é digital, independente da posição | Refaça `CAL`; se persistir, `ZEROCLR` |
| Comando ignorado | Em modo guiado (CAL/STEPPER/…) | Saia com `CANCELAR`/`PARAR` |
