Listagem BallB

Problemas: Dificuldade com a pinagem da Mesa Resitiva, fazia leitura de dados errados, ou parte da mesa não funcionava
Problemas: Necessidade de condençar o hardware de controle em uma PCB para poder realizar a montagem de maneira devida, e sem problemas de ligação, com fios espalhados
Problemas: Impressão 3D relativamente sensivel ao peso dos motores, foi necessário imprimir novamente.

Problematica associada aos materias necessários para a estrutura mecanica: Parafusos Olhais de difícil acesso, o que facilitou foi o professor possuir este parafuso.

Problema tecnico: Foi colocado para fabricar a pcb que iria abrigar os componentes, mas infelizmente existiu um problema na fabricação e ficou a placa toda em curto, e não deu de utilizar a pcb fabricada, tivemos que fazer a soldagem dos componentes em uma placa perfurada, soldando as trilhas.

Além disso, existiu o custo de compra dos componentes. TABELA

---

# Problemas de Firmware e Controle (resolvidos)

Dificuldades encontradas na parte embarcada/controle e como foram resolvidas.

## Sensor / leitura da tela

- **Falso-positivo de toque (bola fantasma):** as duas camadas do painel formam um
  capacitor; com os motores girando, o acoplamento elétrico fazia o ADC ler "alto"
  sem bola. **Solução:** detecção de presença **digital** por pull-up (aterra a
  camada X, flutua Y−, lê Y+ com pull-up; toque puxa a nível baixo), com
  multi-amostragem para rejeitar picos. Imune ao acoplamento.
- **Cantos não detectavam a bola:** a presença usava também um limiar de ADC, mas
  perto das bordas a leitura de *posição* vai legitimamente a ~0, barrando o toque.
  **Solução:** a presença passou a depender **só** da detecção digital (independe
  da posição); o ADC é usado apenas para a coordenada.
- **Leituras espúrias ("a mesa surta do nada"):** um glitch isolado de leitura
  injetava um erro falso enorme. **Solução:** **rejeição de outliers** — saltos
  >30 mm em 10 ms são descartados (mantém o último bom); só são aceitos se
  persistirem (movimento real).
- **Ponto preso (equilibra "o nada"):** painéis com contato espúrio fixo. **Solução:**
  comando `ZERO` (acumula pontos mortos, salvo no NVS) e `ZEROCLR` para limpar.

## Atuadores / motores

- **Motores violentos e passando do ponto:** microstepping do DRV8825 não estava em
  1/16. **Solução:** fixar **M0/M1/M2 = L/L/H** (1/16) — movimento suave; só então
  ajustar velocidade/aceleração.
- **Sentido invertido (`DESCER` subia):** **Solução:** flags `MOTOR_*_INVERT` por
  motor em `steppers.h`.
- **Homing forçando os braços:** **Solução:** a plataforma **sobe** um pouco antes
  de nivelar, garantindo que o 1º movimento seja para cima (longe da base).
- **`STEPPER` não subia até o fim:** saturação cinemática com geometria placeholder.
  **Solução:** calibração de curso reescrita em **espaço de passos** (direto), com
  `MIN`/`MAX` e repouso no meio.

## Malha de controle

- **Mesa equilibra e "surta" do nada:** `Kd` salvo estava ~120× abaixo do
  necessário → malha **sub-amortecida** (a planta é um duplo integrador, §2.3 da
  `TEORIA.md`; o derivativo é quem dá o amortecimento). **Solução:** subir `Kd`
  (damping) + a rejeição de outliers acima.
- **Bola para deslocada e só centra devagar:** base torta (perturbação DC) + `Ki`
  fraco → o integral lento leva muito tempo e zera a cada bola. **Solução:** comando
  **`TRIM`** — *feedforward* de nível salvo no NVS (a plataforma nasce nivelada).
- **Calibração do centro enviesava:** medir o centro com a bola "no olho" introduzia
  erro. **Solução:** o `CAL` passou a usar **só os 4 cantos** e **derivar o centro**
  deles (ponto médio, geometricamente exato).
- **Auto-tune (`PIDAUTO`) pegava ruído / não melhorava:** **Solução:** redesenho
  adaptativo partindo de um baseline seguro, pontuando pelo tempo que a bola fica na
  mesa e pelo erro médio.

## Console / comunicação

- **Flash por JTAG falhava e comandos não chegavam:** a placa usa o **USB-Serial-JTAG**
  nativo, não a UART externa. **Solução:** console mudado para USB-Serial-JTAG; o
  `cmd_task` lê o `stdin`. Ferramenta **`bb_tool`** para build/flash/monitor.

> A fundamentação teórica desses ajustes (duplo integrador, papel do `Kd`,
> integral vs. *feedforward* para a base torta) está na `TEORIA.md` §2, §3 e §10.