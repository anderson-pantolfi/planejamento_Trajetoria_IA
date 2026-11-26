# 🤖 Roteamento Dinâmico com A* para Veículos Autônomos (Pioneer P3DX)

Sistema determinístico de planejamento de trajetória e replanejamento dinâmico em tempo real utilizando o algoritmo de busca informada **A\*** sobre um grid de ocupação 2D, desenvolvido para guiar um robô móvel **Pioneer P3DX** no simulador **CoppeliaSim** frente a interdições imprevistas.

---

## 📋 Sobre o Projeto

Veículos autônomos operando em ambientes dinâmicos não podem depender apenas de rotas estáticas pré-calculadas. Neste projeto, o robô deve navegar de um ponto de origem (**A**) até um destino (**B**) em uma arena fechada contendo obstáculos fixos e **9 cancelas de interdição** acionáveis em tempo real pelo usuário (teclas `1` a `9`).

O robô não conhece previamente o estado das cancelas: ele mantém um **mapa interno parcial** que só é atualizado quando uma cancela entra em seu raio de percepção local ($1,0\text{ m}$), recalculando a rota dinamicamente sempre que o ambiente conhecido sofre alterações.

### Principais Funcionalidades
* **Mapeamento por Grid de Ocupação:** Discretização da arena em uma grade de $114 \times 135$ células ($15.390$ células com resolução de $\approx 0,112\text{ m}$), com suporte a obstáculos rotacionados e inflação pelo raio do robô ($0,22\text{ m}$).
* **Busca Informada A\*:** Planejamento em vizinhança de 8 direções com **heurística octil** admissível e bloqueio de cortes de quina em diagonais.
* **Suavização de Trajetória:** Redução de waypoints via verificação de linha de visada (*line-of-sight*) com margem lateral de segurança.
* **Percepção Local e Esquecimento Otimista:** Atualização dinâmica de barreiras dentro de $1,0\text{ m}$ e limpeza da memória de cancelas distantes ao encontrar becos sem saída[cite: 3].
* **Camada Reativa de Segurança:** Leitura dos 4 sensores ultrassônicos frontais (`ultrasonicSensor[2..5]`) e monitoramento de estagnação física para acionar marcha-ré diferencial e bloqueios temporários de quina ($6,0\text{ s}$)[cite: 3, 5].
* **Benchmarking Integrado:** Comparação simultânea entre **A\*** e **Dijkstra** ($h = 0$) com exportação automática de métricas em `.csv`.

---

## 🛠️ Tecnologias e Bibliotecas

* **Simulador:** [CoppeliaSim](https://www.coppeliarobotics.com/) (comunicação síncrona via ZeroMQ Remote API)
* **Linguagem:** Python 3
* **Dependências:**
  * `coppeliasim-zmqremoteapi-client`.
  * `numpy`.
  * `keyboard`.

---

## ⚙️ Parâmetros do Sistema

| Parâmetro | Valor | Descrição |
| :--- | :--- | :--- |
| **Dimensões da Arena** | $[-6,40; 6,40] \times [-7,56; 7,56]\text{ m}$ | Limites espaciais no referencial global |
| **Resolução do Grid** | $\approx 0,112\text{ m}$ ($114 \times 135$) | Discretização do mapa 2D. |
| **Raio de Inflação** | $0,22\text{ m}$ | Margem de segurança aplicada aos obstáculos.|
| **Raio de Percepção** | $1,0\text{ m}$ | Distância máxima para detectar o estado de uma cancela. |
| **Velocidade Base / Ganho** | $0,6\text{ rad/s}$ / $K = 1,5$ | Controle proporcional diferencial das rodas. |
| **Limiar Ultrassônico** | $0,08\text{ m}$ | Distância frontal mínima para acionar marcha-ré. |
| **Detecção de Contato** | $< 0,025\text{ m}$ em $0,7\text{ s}$ | Janela de verificação de travamento físico. |

---

## 🚀 Como Executar

### 1. Pré-requisitos
Instale as dependências Python necessárias:
```bash
pip install coppeliasim-zmqremoteapi-client numpy keyboard
