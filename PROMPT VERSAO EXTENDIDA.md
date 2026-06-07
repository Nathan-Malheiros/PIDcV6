# Livro Completo de Controle Aplicado ao Projeto Ball Balancer 3RPS

Vou fornecer toda a documentação do meu projeto Ball Balancer 3RPS, incluindo firmware, modelagem matemática, simulações, arquitetura do hardware, apostilas existentes, diagramas, relatórios e códigos-fonte.

Sua tarefa é atuar como um professor universitário especialista em:

* Engenharia de Controle
* Controle Digital
* Sistemas Embarcados
* Robótica
* Mecatrônica
* Controle Moderno
* Modelagem Matemática

e produzir um livro técnico completo em LaTeX (.tex), utilizando o Ball Balancer 3RPS como estudo de caso central.

---

# Objetivo

Não quero apenas uma apostila.

Quero um material equivalente a um pequeno livro universitário.

O leitor deve conseguir sair de um conhecimento básico de cálculo e chegar até a compreensão completa do sistema, incluindo:

* Modelagem física
* Controle clássico
* Controle digital
* Implementação embarcada
* Sintonia
* Estabilidade
* Projeto de controladores
* Limitações reais de hardware

O Ball Balancer deve ser usado como exemplo principal em praticamente todos os capítulos.

---

# Público-Alvo

* Estudantes de Engenharia
* Alunos de TCC
* Desenvolvedores de sistemas embarcados
* Profissionais de automação
* Entusiastas de robótica

O texto deve ser didático, porém tecnicamente rigoroso.

---

# Tamanho Esperado

Entre 120 e 150 páginas.

Caso necessário, ultrapassar ligeiramente esse limite é aceitável.

Priorizar qualidade e profundidade.

---

# Estrutura Desejada

# PARTE I — FUNDAMENTOS MATEMÁTICOS

## Capítulo 1 — Introdução aos Sistemas Dinâmicos

Explicar:

* O que é um sistema dinâmico
* Entradas
* Saídas
* Estados
* Perturbações
* Controle

Utilizar exemplos cotidianos.

---

## Capítulo 2 — Revisão de Cálculo

Explicar:

* Derivadas
* Integrais
* Equações diferenciais
* Interpretação física

Com exemplos gráficos.

---

## Capítulo 3 — Sistemas de Primeira e Segunda Ordem

Explicar:

* Constante de tempo
* Resposta exponencial
* Oscilações

Construir intuição física.

---

# PARTE II — MODELAGEM DO BALL BALANCER

## Capítulo 4 — O Problema do Ball Balancer

Explicar:

* História
* Aplicações
* Desafios

---

## Capítulo 5 — Modelagem Física da Esfera

Derivar completamente:

x¨ = (5/7)g sin(α)

Mostrar:

* Newton
* Euler
* Energia

---

## Capítulo 6 — Linearização

Explicar:

* Série de Taylor
* Pequenos ângulos
* Limitações

---

## Capítulo 7 — Identificação Experimental

Explicar:

* Como validar o modelo
* Comparação entre teoria e hardware

---

# PARTE III — CONTROLE CLÁSSICO

## Capítulo 8 — Transformada de Laplace

Explicar profundamente:

* Motivação
* Propriedades
* Aplicações

---

## Capítulo 9 — Funções de Transferência

Explicar:

* Polos
* Zeros
* Ganhos

Aplicados ao Ball Balancer.

---

## Capítulo 10 — Diagramas de Blocos

Mostrar:

* Sistema físico
* Sistema simplificado
* Malha aberta
* Malha fechada

---

## Capítulo 11 — Estabilidade

Explicar:

* Critério dos polos
* Routh-Hurwitz
* Interpretação física

---

## Capítulo 12 — Lugar das Raízes (Root Locus)

Explicar:

* Construção
* Interpretação
* Influência dos ganhos

Aplicar ao Ball Balancer.

---

## Capítulo 13 — Resposta em Frequência

Explicar:

* Diagramas de Bode
* Magnitude
* Fase

Aplicar ao sistema.

---

# PARTE IV — CONTROLADOR PID

## Capítulo 14 — Controle Proporcional

---

## Capítulo 15 — Controle Integral

---

## Capítulo 16 — Controle Derivativo

---

## Capítulo 17 — PID Completo

Explicar:

* Interpretação matemática
* Interpretação física
* Analogia mola-amortecedor-memória

---

## Capítulo 18 — Sintonia de PID

Explicar:

* Ziegler-Nichols
* Cohen-Coon
* Alocação de polos

---

## Capítulo 19 — Overshoot, ζ e ωn

Explicar profundamente:

* Sobressinal
* Tempo de subida
* Tempo de acomodação
* Frequência natural
* Amortecimento

---

# PARTE V — CONTROLE DIGITAL

## Capítulo 20 — Fundamentos do Controle Digital

Explicar:

* Amostragem
* Quantização
* Nyquist
* Aliasing

---

## Capítulo 21 — Transformada Z

Explicar profundamente:

* Conceito
* Propriedades
* Relação com Laplace

---

## Capítulo 22 — Discretização

Explicar:

* Euler Forward
* Euler Backward
* Tustin

---

## Capítulo 23 — PID Discreto

Derivar completamente.

---

## Capítulo 24 — Efeitos da Frequência de Amostragem

Explicar:

* 10 Hz
* 50 Hz
* 100 Hz
* 500 Hz

Comparando estabilidade e desempenho.

---

# PARTE VI — IMPLEMENTAÇÃO REAL

## Capítulo 25 — Arquitetura do Firmware

Analisar:

* control.c
* pid.c
* kinematics.c

---

## Capítulo 26 — Sensor Resistivo

Explicar:

* Funcionamento
* Calibração
* Conversão para mm

---

## Capítulo 27 — Filtragem

Explicar:

* Filtro derivativo
* Média móvel
* IIR

---

## Capítulo 28 — Anti-Windup

Explicar profundamente.

---

## Capítulo 29 — Saturações e Limitações Reais

Explicar:

* Motores
* Curso mecânico
* Perda de passos
* Velocidade máxima

---

## Capítulo 30 — Atrasos e Latências

Explicar:

* Sensor
* Processamento
* Atuadores

---

# PARTE VII — CONTROLE MODERNO E EVOLUÇÕES

## Capítulo 31 — Espaço de Estados

Explicar:

ẋ = Ax + Bu

y = Cx + Du

Aplicado ao Ball Balancer.

---

## Capítulo 32 — Observadores

Introdução conceitual.

---

## Capítulo 33 — Filtro de Kalman

Introdução conceitual.

---

## Capítulo 34 — Controle Ótimo (LQR)

Mostrar como o Ball Balancer poderia utilizar LQR.

---

## Capítulo 35 — MPC

Introdução conceitual.

---

# PARTE VIII — GUIA DE ENGENHARIA

## Capítulo 36 — Como Fazer a Sintonia na Bancada

Passo a passo.

---

## Capítulo 37 — Diagnóstico de Problemas

Tabela completa:

* Oscilação
* Instabilidade
* Saturação
* Ruído
* Atrasos
* Sensor

---

## Capítulo 38 — Perguntas de Banca

Criar uma seção simulando perguntas difíceis de professores.

Responder tecnicamente.

---

## Capítulo 39 — Conclusões

Conectar toda a teoria ao projeto.

---

# Recursos Visuais Obrigatórios

Utilizar:

* TikZ
* PGFPlots
* Diagramas de blocos
* Root Locus
* Diagramas de Bode
* Diagramas de polos
* Fluxogramas
* Quadros de observação
* Quadros de curiosidade
* Quadros de aplicação prática

---

# Recursos Didáticos Obrigatórios

Ao longo do texto inserir:

### Intuição Física

Explicações simples.

### Aplicação no Projeto

Conectar teoria ao Ball Balancer.

### Erros Comuns

Mostrar armadilhas frequentes.

### Pergunta de Banca

Questões típicas de professores.

### Resumo do Capítulo

Ao final de cada capítulo.

---

# Bibliografia

Utilizar referências clássicas:

* Ogata
* Nise
* Franklin, Powell & Emami-Naeini
* Dorf & Bishop
* Åström & Murray
* Kuo

Gerar arquivo BibTeX completo.

---

# Formato de Saída

Retorne exclusivamente o código LaTeX completo.

O projeto deve compilar diretamente no Overleaf.

O resultado final deve parecer um livro técnico profissional de Engenharia de Controle, utilizando o Ball Balancer 3RPS como estudo de caso principal em toda a obra.
