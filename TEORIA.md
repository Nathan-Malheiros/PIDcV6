# Teoria de Controle — Ball Balancer 3RPS

Documento de fundamentação teórica do projeto, voltado a **relatório e
apresentação**. Cobre o modelo da planta, a função de transferência, o projeto
do PID, **a dedução de cada ganho (Kp, Ki, Kd)** e a discretização (controle
digital) — tudo amarrado ao código real do firmware (`main/pid.c`,
`main/control.c`) e da simulação (`tools/PIDSimba.py`).

> Convenção de notação: variáveis físicas em mm e segundos. O eixo X e o eixo Y
> são **idênticos e independentes** (dois PIDs iguais, um por eixo), então toda a
> análise é feita para **um eixo** e replicada.

---

## 1. O problema de controle

Uma bola apoiada sobre uma superfície que pode ser inclinada. Inclinando a mesa
de um pequeno ângulo, a gravidade gera uma força que acelera a bola. O objetivo
é manter a bola numa posição de referência (o centro), apesar de perturbações
(empurrões, imperfeições mecânicas, ruído do sensor).

```
   referência r ──►(+)── e ──► [ PID ] ──► u (inclinação) ──► [ PLANTA ] ──► x (posição)
                    ▲−                                                         │
                    └─────────────────── sensor (tela resistiva) ◄────────────┘
```

- **`e = x − r`** : erro de posição (no firmware, `err = meas − setpoint`, em mm).
- **`u`** : saída do controlador = componente do **vetor normal** da mesa (`nx`/`ny`),
  que para pequenos ângulos equivale à **inclinação (slope)** do plano.
- **Planta** : dinâmica da bola rolando + cinemática + atuadores.

---

## 2. Modelo da planta (bola rolando num plano inclinado)

### 2.1. Dinâmica de Newton–Euler

Para uma **esfera maciça** que rola **sem deslizar** sobre um plano inclinado de
ângulo `α`, somam-se a 2ª lei de Newton (translação) e a equação de Euler (rotação):

$$ m\,\ddot{x} = m g \sin\alpha - F_{at}, \qquad I\,\dot\omega = F_{at}\,r, \qquad \dot{x} = \omega r $$

Para a esfera maciça, o momento de inércia é `I = (2/5) m r²`. Eliminando a força
de atrito `F_at` e usando `ẍ = r ω̇`:

$$ \left(m + \frac{I}{r^2}\right)\ddot{x} = m g \sin\alpha
   \;\Longrightarrow\; \frac{7}{5}\,\ddot{x} = g\sin\alpha $$

$$ \boxed{\;\ddot{x} = \frac{5}{7}\, g \sin\alpha \;\approx\; \frac{5}{7}\, g\,\alpha\;} $$

A fração **5/7 ≈ 0,714** é exatamente a constante `ROLL_FACTOR = 5.0/7.0` usada
na simulação física (`tools/PIDSimba.py`). A aproximação `sin α ≈ α` vale para os
pequenos ângulos de operação (a saída é limitada a `|u| ≤ 0,25 rad ≈ 14°`).

### 2.2. Relação entre a saída do controlador e o ângulo

A saída do PID, `u = nx`, é a componente x do vetor normal unitário da mesa.
Para pequenas inclinações, **`nx ≈ tan α ≈ α`** (a inclinação do plano). Logo,
usando `g = 9810 mm/s²`:

$$ \ddot{x} \;=\; \underbrace{\tfrac{5}{7}\,g}_{A}\; u,
   \qquad A = \tfrac{5}{7}\times 9810 \approx 7007\ \text{mm/s}^2 $$

### 2.3. Função de transferência da planta

Aplicando Laplace com condições iniciais nulas a `ẍ = A·u`:

$$ \boxed{\;P(s) = \frac{X(s)}{U(s)} = \frac{A}{s^2},\qquad A \approx 7007\ \text{mm/s}^2\;} $$

A planta é um **duplo integrador**. Implicações de projeto:

- **Tipo 2** ⇒ erro estacionário **nulo** a degrau e a rampa de referência.
- Os dois polos estão **na origem** (`s = 0`): a planta é **marginalmente estável**.
  Realimentação puramente proporcional coloca os polos de malha fechada sobre o
  eixo imaginário → **oscilação não amortecida**. É por isso que o termo
  **derivativo é obrigatório** (ele injeta a fase/amortecimento que falta).

---

## 3. O controlador PID

### 3.1. Forma contínua

$$ u(t) = K_p\,e(t) + K_i\!\int_0^t\! e(\tau)\,d\tau + K_d\,\dot{e}(t) $$

$$ C(s) = K_p + \frac{K_i}{s} + K_d\,s $$

No firmware (`main/pid.c`) o derivativo é **filtrado** por um passa-baixa de 1ª
ordem (constante `d_tau = 0,03 s`) para não amplificar o ruído do sensor:

$$ C(s) = K_p + \frac{K_i}{s} + \frac{K_d\,s}{1 + \tau_d\,s},\qquad \tau_d = 0{,}03\ \text{s} $$

### 3.2. Papel de cada termo nesta planta

| Termo | Função aqui | Consequência de exagerar |
|---|---|---|
| **P** (`Kp`) | Define a rigidez/velocidade da resposta (`ωn`). Sozinho → oscila. | Oscilação crescente |
| **D** (`Kd`) | **Essencial**: adiciona amortecimento (freia pela velocidade da bola). | Lentidão, sensível a ruído |
| **I** (`Ki`) | Remove desvio residual por assimetria mecânica / mesa fora de nível. | Windup, instabilidade lenta |

---

## 4. Malha fechada e mapeamento para 2ª ordem

Ignorando temporariamente o filtro do derivativo e o integral (`Ki` é pequeno —
ver §6.3), a malha fechada com `P(s) = A/s²` e o trecho **PD** dá:

$$ \frac{X(s)}{R(s)} = \frac{C(s)P(s)}{1 + C(s)P(s)}
   \;\Rightarrow\; \text{equação característica: } \; s^2 + A K_d\,s + A K_p = 0 $$

Comparando com a forma canônica de 2ª ordem `s² + 2ζωₙs + ωₙ²`:

$$ \boxed{\;\omega_n = \sqrt{A\,K_p}\;}\qquad
   \boxed{\;\zeta = \frac{K_d}{2}\sqrt{\frac{A}{K_p}}\;} $$

- `ωn` (frequência natural) ⇒ **quão rápido** a bola converge.
- `ζ` (amortecimento) ⇒ **overshoot**: `ζ<1` oscila, `ζ=1` crítico (sem overshoot),
  `ζ>1` sobreamortecido (lento).
- Tempo de acomodação (2 %): `t_s ≈ 4 / (ζ ωn)`.

---

## 5. Projeto dos ganhos por alocação de polos (método do TCC)

A simulação do `PIDSimba.py` segue o método do TCC de **Giovanna F. Cordeiro
(UNESP Ilha Solteira, 2022) — "Controle Clássico da Planta Ball Balancer"**, que
projeta o controlador da mesa por **alocação de polos**. A planta é o duplo
integrador `X(s)/α(s) = A/s²` (§2–3); o controlador é um PID. A malha fechada é de
**3ª ordem**, então alocamos os polos como um par dominante de 2ª ordem mais um
polo real:

$$ (s^2 + 2\zeta\omega_n s + \omega_n^2)\,(s + p_o) $$

### 5.1. Dos requisitos a `ζ` e `ωn`

Requisitos de projeto (do TCC, para a mesa): overshoot `PO ≤ 7,5 %`, tempo de
acomodação `t_s ≤ 2,5 s` (faixa `cts = 0,04`). Pelas fórmulas de 2ª ordem (Ogata):

$$ \zeta = \sqrt{\frac{\ln^2(PO/100)}{\pi^2 + \ln^2(PO/100)}} \approx 0{,}636 $$

$$ \omega_n = -\frac{\ln\!\big(cts\sqrt{1-\zeta^2}\big)}{t_s\,\zeta} \approx 2{,}19\ \text{rad/s} $$

### 5.2. De `ζ, ωn` aos ganhos (igualando os coeficientes)

A equação característica da malha externa (PID sobre `A/s²`) é
`s³ + A·Kd·s² + A·Kp·s + A·Ki = 0`. Igualando aos coeficientes do polinômio
desejado (com `po = 1`, um polo real "rápido"):

$$ A K_d = 2\zeta\omega_n + p_o,\quad
   A K_p = \omega_n^2 + 2\zeta\omega_n p_o,\quad
   A K_i = \omega_n^2 p_o $$

Resolvendo para `A = (5/7)·9810 = 7007 mm/s²` (posição em mm, `α` em rad):

| Ganho | Fórmula | "PID inicial" |
|---|---|---|
| `Kd` | `(2ζωn + po) / A` | `5,40×10⁻⁴` |
| `Kp` | `(ωn² + 2ζωn·po) / A` | `1,08×10⁻³` |
| `Ki` | `ωn²·po / A` | `6,83×10⁻⁴` |

### 5.3. "PID melhorado" — redução do overshoot (como no TCC)

O PID de alocação de polos acima é o **"PID inicial"**: ele tem ação integral
forte e, no laço fechado de 3ª ordem, dá **overshoot ~25–30 %** (o TCC obteve
23,4 % e o nosso validador `sim_pid.py` confirma ~29 %). Seguindo o mesmo
procedimento do TCC ("PID melhorado": aumentar `Kd` e apertar a ação integral),
**reduzimos `Ki` e aumentamos `Kd`** para amortecer (`ζ ≈ 1,15`), mantendo erro
residual nulo:

→ valores adotados em `PIDSimba.py`: **`SIM_KP=1,08×10⁻³`, `SIM_KI=3,0×10⁻⁴`,
`SIM_KD=9,0×10⁻⁴`** (com `α` interpretado como a componente da normal `n ≈ α`).

**Validação (teste headless, sem hardware):** com esses ganhos a bola **converge
ao centro a partir dos quatro cantos**, overshoot ≈ 15 %, erro residual final
≈ 1 mm (→ 0 com o tempo, pela ação integral). **A teoria fecha com o código.**

### 5.4. Estrutura em cascata e adaptação 2-servos → 3-servos

O TCC controla um módulo Quanser **2 DOF** (2 servomotores, um por eixo) em
**cascata**:

```
Xd ─►(+)─e─►[ Cbb (PID) ]─θd─►(+)─►[ Cs (PD) ]─V─►[ motor ]─θ─►[ bola ]─►X
       ▲−                    ▲−                                         │
       └── posição da bola ──┴───── ângulo do motor ──────────────────┘
```

- **Malha externa** `Cbb`: do erro de posição calcula a **inclinação desejada** da mesa.
- **Malha interna** `Cs`: leva o motor ao ângulo desejado (rápida, `t_p ≤ 0,15 s`).

**Nosso rig é 3RPS (3 motores).** A adaptação: a inclinação desejada `(αx, αy)`
não vira um único ângulo de servo, e sim **três** ângulos `(θA, θB, θC)` pela
**cinemática inversa 3RPS** (`machine_theta`, §7). A malha interna então move os
3 motores para realizar aquela inclinação. No `PIDSimba.py`:

- malha externa = PID de posição → `αx, αy` (ganhos de §5.2);
- malha interna = a inclinação efetiva persegue a desejada (dinâmica dos motores,
  `TILT_TAU = 0,05 s`), e os 3 alvos saem de `machine_theta(αx, αy)`.

A física integra a bola **no mesmo plano** que é desenhado (`x'' = −(5/7)g·∇z`),
garantindo que a bola role *ladeira abaixo do tampo que você vê* e a mesa se
incline para **trazê-la ao centro** (realimentação negativa).

### 5.5. Os ganhos *padrão do firmware* são diferentes — e por quê

`main/control.c` parte de **`KP=8×10⁻⁴`, `KI=2×10⁻⁵`, `KD=1,2×10⁻²`**. Pela
análise linear isso dá `ζ ≈ 17,8` (fortemente sobreamortecido). Não é um erro: é
uma escolha **conservadora e robusta** para o *hardware real*, por dois motivos:

1. **Saturação proposital do derivativo**: com a saída limitada a `±0,25`, o termo
   D sozinho satura quando `Kd·v ≥ 0,25`, isto é, a partir de `v ≈ 0,25/0,012 ≈ 21 mm/s`.
   Acima disso o D atua como **freio de velocidade quase bang-bang** — muito eficaz
   contra empurrões e imune a pequenas variações de ganho. (Esse `~21 mm/s` está
   anotado no comentário de `pid.c`.)
2. **Ponto de partida seguro**: o firmware é tunado **no rig** com o PID Tuner ao
   vivo (subir `Kp` até oscilar, adicionar `Kd`, depois um toque de `Ki`). Os
   defaults precisam ser mansos para não derrubar a bola na primeira energização.

> Resumo: os ganhos de **simulação** (§5.1–5.3) são projetados analiticamente
> contra o modelo ideal; os de **firmware** são conservadores e refinados
> experimentalmente. As duas escolhas vêm da mesma teoria.

---

## 6. Controle digital (discretização)

O laço roda a **`100 Hz`** (`LOOP_HZ = 100`), logo `T = 0,01 s`. A frequência de
Nyquist é 50 Hz ≫ `ωn/2π ≈ 0,23 Hz`, então a amostragem é folgada e a aproximação
contínua de §4–5 é válida. (O laço subiu de 50 → 100 Hz para reduzir o atraso de
amostragem pela metade e melhorar o tempo de resposta.)

### 6.1. Integral — Euler para a frente

$$ I[k] = I[k-1] + e[k]\,T \quad\Longleftrightarrow\quad s \to \frac{z-1}{T} $$

No código: `p->integ += err * dt;`

### 6.2. Derivada — diferença regressiva + filtro passa-baixa

Derivada bruta por diferença regressiva, depois suavizada por um IIR de 1ª ordem:

$$ d[k] = \frac{e[k]-e[k-1]}{T}, \qquad
   d_{lpf}[k] = d_{lpf}[k-1] + \alpha\big(d[k]-d_{lpf}[k-1]\big) $$

com o coeficiente derivado de `τd`:

$$ \alpha = \frac{T}{\tau_d + T} = \frac{0{,}01}{0{,}03+0{,}01} = 0{,}25 $$

No código (`pid.c`): `float a = dt/(d_tau+dt); d_lpf += a*(deriv − d_lpf);`.
O filtro corta ruído acima de `1/(2πτd) ≈ 5,3 Hz`, preservando a dinâmica da bola.
(A 100 Hz, `α` cai de 0,4 → 0,25: mais suavização por amostra, mesma `τd`.)

### 6.3. Anti-windup por *back-calculation*

Quando a saída satura no limite `±0,25`, o integrador é **puxado de volta** pela
quantidade exata que excedeu, evitando acúmulo (windup) que causaria overshoot
gigante ao sair da saturação:

$$ \text{se } u>u_{max}:\quad I \mathrel{-}= \frac{u-u_{max}}{K_i},\quad u=u_{max} $$

No código (`pid.c`): `if (out>out_max){ p->integ -= (out-out_max)/p->ki; out=out_max; }`.

### 6.4. Lei de controle discreta completa

$$ \boxed{\;u[k] = K_p\,e[k] + K_i\,I[k] + K_d\,d_{lpf}[k]\;}
   \quad\text{seguida de saturação + anti-windup} $$

A cada borda de "sem toque", o controlador é **resetado** (`pid_reset`) para não
arrastar estado integral/derivativo de uma sessão para a próxima.

---

## 7. Da inclinação aos motores — cinemática inversa 3RPS

O PID produz um **vetor de inclinação** `(nx, ny)`; a mesa é movida por 3 motores
em arranjo **3RPS** (3× Revolute-Prismatic-Spherical). A função
`machine_theta(perna, hz, nx, ny)` (`main/kinematics.c`, porte fiel do projeto do
Aaed Musa) resolve, em forma fechada, o **ângulo de cada motor** que realiza aquela
inclinação:

$$ (n_x, n_y) \;\xrightarrow{\text{cinemática inversa}}\; (\theta_A,\theta_B,\theta_C) $$

Do ponto de vista de controle, essa cinemática é um **mapa estático não-linear**
(sem dinâmica própria significativa): ela não altera a *ordem* do laço, apenas
traduz o comando de inclinação em posições angulares. Por isso a análise linear
de §4 (feita no domínio da inclinação) permanece válida em 1ª ordem.

A conversão final para passos:

$$ \text{passos} = (\theta_{orig} - \theta)\cdot\frac{200\cdot\text{microsteps}}{360°},
   \qquad \theta_{orig} = \texttt{machine\_theta}(\cdot,\ n_x{=}0,\ n_y{=}0) $$

---

## 8. O atuador — motores de passo com perfil trapezoidal

Cada motor segue a posição-alvo com um **perfil trapezoidal de velocidade**
(`main/steppers.c`, timer de 10 kHz): acelera até `max_speed` e desacelera a tempo
de parar **exatamente** no alvo. A velocidade máxima da qual ainda se consegue
frear a tempo é:

$$ v_{lim} = \sqrt{2\,a\,d} $$

onde `d` é a distância restante (passos) e `a` a aceleração. Sem essa rampa, uma
partida brusca causaria **perda de passos** (o motor "escorrega"). Em equilíbrio
usa-se `1800 passos/s` e `16000 passos/s²` (suaves, pressupõem o DRV8825 em
**1/16** — M0/M1/M2 = L/L/H); no homing, valores menores.

O atuador adiciona um pequeno atraso de fase, mas sua banda (centenas de Hz) é
muito superior à dinâmica da bola (~0,23 Hz), então não compromete a estabilidade
projetada.

---

## 9. Estabilidade e margens

- **Planta `A/s²`** isolada: marginalmente estável (2 polos na origem).
- **Com PD**: equação característica `s² + A Kd s + A Kp = 0`. Pelo critério de
  Routh–Hurwitz, todos os coeficientes são positivos ⇒ **malha estável** para
  quaisquer `Kp, Kd > 0`. O derivativo é o que garante a estabilidade.
- **Com o I** (pequeno): adiciona um polo lento em malha aberta; mantido com `Ti`
  longo (25–40 s) para não reduzir a margem de fase de forma relevante.
- **Amostragem**: `T = 0,01 s` (100 Hz) introduz atraso ≈ `T/2 = 0,005 s`,
  desprezível frente a `1/ωn ≈ 0,7 s`.

---

## 10. Procedimento prático de sintonia (no hardware)

Feito ao vivo pelo **terminal serial** (`PID`/`PID SAVE`, sem rebuild — ver
`SERIAL_COMMANDS.md`). Ligue o monitor de erro com **`ERROR`** para acompanhar
`ex/ey/dist` a cada segundo:

1. Zere `Ki` e `Kd`. Suba `Kp` até a bola **oscilar** de forma sustentada em torno
   do centro (ganho crítico `Ku`, período `Tu`).
2. Adicione `Kd` para **amortecer** a oscilação (alvo: sem overshoot ⇒ `ζ ≈ 1`).
   **Num duplo integrador este é o passo que estabiliza** (§2.3): com `Kd` baixo
   demais a malha fica sub-amortecida e qualquer perturbação vira uma excursão
   grande ("a mesa surta do nada").
3. Adicione um **toque** de `Ki` só para eliminar o desvio residual (mesa torta).
4. `PID SAVE` grava no **NVS** — persiste entre reinicializações.

Como ponto de partida quantitativo, Ziegler–Nichols clássico sugere
`Kp ≈ 0,6 Ku`, `Ti ≈ 0,5 Tu`, `Td ≈ 0,125 Tu` — útil como chute inicial antes do
ajuste fino acima.

### 10.1. Robustez de leitura (rejeição de outliers)

Uma única leitura espúria do painel (acoplamento dos motores, repique do contato,
amostra de ADC suja) injeta um **erro falso e enorme** que, derivado, dá um
"chute" na saída — a mesa pode ir ao talo mesmo com a bola parada. O firmware
**rejeita saltos impossíveis**: se a posição pula mais de `30 mm` entre amostras
(10 ms), o valor é descartado e mantém-se o último bom; se o salto **persistir**
por ~3 amostras (movimento real ou recolocação), é aceito (resync). A bola real
anda no máx. ~8 mm/amostra, então isso só filtra glitch — sem atrasar o controle
(`touch_screen.c`).

### 10.2. Compensação da base torta — *feedforward* de nível (`TRIM`)

A estrutura inteira ficar levemente **torta** equivale a uma **perturbação
constante (DC)** na entrada da planta: a plataforma, na posição "nivelada" dos
motores, está na verdade inclinada de `θ_base`, e a bola escorrega para um lado.
Pela §3.2, **só o termo integral** zera erro de regime — ele acumula até comandar
a inclinação `−θ_base` que mantém a bola no centro.

O problema é a **velocidade**: com `Ki` pequeno (longo `Ti`), o integral leva
dezenas de segundos a montar essa correção, e como ele **zera a cada bola**
(`pid_reset`), o desvio reaparece a cada recolocação. A solução adotada é um
**viés de nível** (comando `TRIM`): transfere-se a inclinação constante que o
integral descobriu para um *feedforward* fixo, salvo no NVS e aplicado já no boot
(inclusive em IDLE). Assim a plataforma **nasce nivelada em relação à gravidade**,
a bola começa no centro, e o integral cuida apenas do resíduo. Formalmente, é
separar a rejeição da perturbação DC (feedforward, `θ_ff = −θ_base`) da regulação
em torno dela (PID), em vez de exigir do integral lento as duas coisas.

> **Ganhos atuais do firmware:** `Kp = 8×10⁻⁴`, `Ki = 2×10⁻⁵`,
> `Kd = 1,2×10⁻²`, sinais `−1/−1`, com a rejeição de outliers e o `TRIM`
> compensando a base torta. São os **defaults compilados** (§5.5), refinados no
> hardware e persistidos no NVS — o `Kd` alto é justamente o que amortece o duplo
> integrador (compare com a saturação proposital do derivativo, §6.3).

---

## 11. Tabela-resumo dos parâmetros

| Símbolo | Onde | Valor | Origem |
|---|---|---|---|
| `A = (5/7)g` | planta | ≈ 7007 mm/s² | Newton–Euler (esfera maciça) |
| `T` | laço | 0,01 s (100 Hz) | `LOOP_HZ` |
| `τd` | filtro D | 0,03 s | `PID_D_TAU` |
| `α` | filtro D discreto | 0,25 | `T/(τd+T)` |
| `SIM_KP` | simulação | 1,08×10⁻³ | alocação de polos (§5.2) |
| `SIM_KI` | simulação | 3,0×10⁻⁴ | "PID melhorado" (§5.3) |
| `SIM_KD` | simulação | 9,0×10⁻⁴ | "PID melhorado" (§5.3) |
| `KP` | firmware | 8,0×10⁻⁴ | conservador, tunado no rig (§5.5) |
| `KI` | firmware | 2,0×10⁻⁵ | `Ti = 40 s` |
| `KD` | firmware | 1,2×10⁻² | freio com saturação proposital (§5.5) |
| `u_max` | saturação | ±0,26 | `TILT_LIMIT` |
| `ζ` (sim) | projeto | ≈ 0,636 | `PO ≤ 7,5 %` (§5.1) |
| `ωn` (sim) | projeto | ≈ 2,19 rad/s | `t_s ≤ 2,5 s` (§5.1) |
| `t_s` (sim) | resposta | ≈ 3 s | converge a 0 mm dos 4 cantos (§5.3) |

---

## 12. Referências

- **CORDEIRO, Giovanna Ferreira. *Controle Clássico da Planta Ball Balancer.*
  Trabalho de Graduação, Engenharia Elétrica, UNESP/FEIS, Ilha Solteira, 2022.**
  — base do modelo em cascata, do equacionamento da planta (`Pbb=Kbb/s²`) e do
  projeto dos ganhos por alocação de polos (§5). Adaptado aqui de 2 servos (módulo
  Quanser 2 DOF) para 3 servos (arranjo 3RPS).
- Aaed Musa, *Ball Balancer* — Instructables: <https://www.instructables.com/Ball-Balancer/>
  (cinemática 3RPS e arquitetura original; código em [docs/referencia/](docs/referencia/)).
- Ogata, *Engenharia de Controle Moderno* — 2ª ordem, Routh, alocação de polos, discretização.
- Franklin, Powell, Workman, *Digital Control of Dynamic Systems* — Euler/Tustin,
  projeto no domínio `z`.
