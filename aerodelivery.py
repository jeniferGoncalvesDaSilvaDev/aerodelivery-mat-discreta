"""
AeroDelivery - protótipo simples (Python + pygame, 100% orientado a objetos)
============================================================================

Implementa o pipeline descrito na seção 5 do documento do projeto:

    Sensores -> Filtro de Kalman -> Cadeia de Markov -> Pesos
             -> Custo adaptativo -> A* -> Melhor rota

Arquitetura (cada responsabilidade em uma classe):

    Config            Constantes e parâmetros do protótipo.
    AdverseCondition  Condição adversa k (vento, chuva...): estados, penalidades,
                      matriz de transição T_k, estado real oculto e sensor ruidoso.
    MovingObstacle    Evento dinâmico: obstáculo móvel (ex.: pássaro, outro drone).
    Environment       Mapa em grade (grafo), eventos estáticos/dinâmicos e sensores.
    Agent             O drone. Possui os métodos kalman_filter, markov_chain e a_star.
    Renderer          Toda a parte visual (pygame).
    Game              Laço principal, entrada do usuário e despacho de entregas.

Como executar:
    pip install pygame
    python aerodelivery.py

Controles:
    ESPAÇO   pausa / continua
    R        força um replanejamento com o A*
    T        mostra/oculta o estado REAL das condições (o agente não o enxerga)
    CLIQUE   cria/remove um prédio (evento estático) na célula clicada
    ESC      sai
"""

from __future__ import annotations

import heapq
import itertools
import random
import sys

import pygame

# Uma célula da grade é (coluna, linha). Cada célula é um vértice do grafo;
# as arestas ligam células vizinhas nas 4 direções.
Cell = tuple[int, int]


# ============================================================================
# CONFIGURAÇÃO
# ============================================================================
class Config:
    """Agrupa todas as constantes do protótipo, para facilitar os ajustes."""

    # --- Mapa -------------------------------------------------------------
    # '#' prédio/obstáculo fixo (aresta bloqueada, custo infinito)
    # '~' zona perigosa (evento estático: penalidade fixa no custo)
    # 'R' restaurante (origem)   'C' cliente (possível destino)   '.' livre
    MAP = [
        "R.....#.....~~......",
        "..##..#..#..~~.##.C.",
        "..##.....#.....##...",
        "......##...###....C.",
        ".~~...##...#........",
        ".~~.......##..#.###.",
        "....####......#.....",
        "........#..~~.......",
        "..###...#..~~..###..",
        "..###......#......C.",
        "......~~...#..##....",
        "..##..~~.......##...",
        "..##....####........",
        "C...................",
    ]
    RISK_PENALTY = 2.0          # custo de entrar numa zona perigosa (normal = 1)
    # Retângulos na ordem de CONDITIONS (x_min, y_min, x_max, y_max), exclusivo.
    WEATHER_ZONES = [(10, 0, 16, 6), (0, 8, 7, 14), (7, 3, 12, 9)]

    # --- Tela -------------------------------------------------------------
    CELL = 40                   # tamanho da célula em pixels
    PANEL_W = 340               # largura do painel lateral
    FPS = 60

    # --- Tempo da simulação (segundos) -------------------------------------
    ENV_TICK = 1.0              # intervalo de atualização do ambiente (pipeline)
    MOVE_INTERVAL = 0.28        # tempo para o drone andar uma célula

    # --- Agente -----------------------------------------------------------
    DELTA = 3                   # horizonte Δ da projeção de Markov (passos)
    TAU = 0.12                  # limiar τ para replanejar (variação de C_pres)
    Q = 0.02                    # variância do modelo Q (Kalman, passo 1)

    # --- Sensores e eventos dinâmicos -------------------------------------
    SENSOR_VARIANCE = 0.05      # variância R do ruído do sensor
    N_OBSTACLES = 3             # quantidade de obstáculos móveis
    OBSTACLE_MOVE_PROB = 0.5    # chance de um obstáculo se mover a cada tick

    # --- Condições adversas (seção 5.2 / 5.7 do documento) -----------------
    # Estado 0 é sempre a situação normal, com penalidade ℓ = 1.
    # Cada linha de 'transition' soma 1 (propriedade de uma Cadeia de Markov).
    CONDITIONS = [
        {   # A matriz do vento é exatamente a Tabela 2 do documento.
            "name": "Vento",
            "labels": ["Calmo", "Moderado", "Forte"],
            "penalties": [1.0, 1.5, 2.5],
            "transition": [[0.7, 0.2, 0.1],
                           [0.3, 0.5, 0.2],
                           [0.2, 0.4, 0.4]],
        },
        {
            "name": "Chuva",
            "labels": ["Sem chuva", "Garoa", "Tempestade"],
            "penalties": [1.0, 1.4, 2.2],
            "transition": [[0.80, 0.15, 0.05],
                           [0.30, 0.50, 0.20],
                           [0.20, 0.40, 0.40]],
        },
        {
            "name": "Neblina",
            "labels": ["Boa vis.", "Neblina", "Densa"],
            "penalties": [1.0, 1.3, 1.8],
            "transition": [[0.85, 0.10, 0.05],
                           [0.25, 0.55, 0.20],
                           [0.15, 0.35, 0.50]],
        },
    ]

    # --- Cores ------------------------------------------------------------
    BG = (24, 26, 33)
    GRID = (44, 48, 60)
    FREE = (52, 58, 72)
    WALL = (20, 21, 26)
    RISK = (120, 78, 40)
    WEATHER_COLORS = [(55, 105, 155), (55, 125, 95), (125, 125, 105)]
    ROUTE = (90, 200, 255)
    DRONE = (255, 255, 255)
    RESTAURANT = (230, 80, 80)
    CLIENT = (90, 210, 130)
    OBSTACLE = (180, 100, 220)
    TEXT = (225, 228, 235)
    MUTED = (140, 146, 160)
    BAR = (90, 200, 255)
    BAR_BG = (38, 42, 54)
    TRUTH = (255, 200, 60)


# ============================================================================
# CONDIÇÃO ADVERSA (Cadeia de Markov + sensor ruidoso)
# ============================================================================
class AdverseCondition:
    """Uma condição adversa k do ambiente (ex.: vento).

    Do lado do *ambiente*, ela evolui como uma Cadeia de Markov: a cada tick o
    estado real muda segundo a matriz de transição ``T_k``. Esse estado real é
    OCULTO para o agente, que só o conhece por meio do sensor (``sense``), que é
    ruidoso.

    Atributos públicos (o "manual do sensor", conhecido pelo agente):
        name, labels, penalties (ℓ_k,s), transition (T_k), sensor_variance (R)
    """

    def __init__(self, name: str, labels: list[str], penalties: list[float],
                 transition: list[list[float]], sensor_variance: float,
                 rng: random.Random) -> None:
        m = len(labels)
        assert len(penalties) == m and len(transition) == m
        assert penalties[0] == 1.0, "o estado normal (0) deve ter ℓ = 1"
        assert all(abs(sum(row) - 1.0) < 1e-9 for row in transition), \
            "cada linha da matriz de transição deve somar 1"

        self.name = name
        self.labels = labels
        self.penalties = penalties
        self.transition = transition
        self.sensor_variance = sensor_variance
        self._rng = rng
        self._true_state = 0                      # estado real (oculto)

    @property
    def n_states(self) -> int:
        """Número de estados m da condição."""
        return len(self.labels)

    @property
    def true_state(self) -> int:
        """Estado real. Uso restrito à visualização/depuração (não ao agente)."""
        return self._true_state

    def step(self) -> None:
        """Avança o estado real um passo, sorteando a linha da matriz T_k."""
        r, acc = self._rng.random(), 0.0
        for nxt, prob in enumerate(self.transition[self._true_state]):
            acc += prob
            if r <= acc:
                self._true_state = nxt
                return

    def sense(self) -> list[float]:
        """Lê o sensor e devolve a medida z_k,s de cada estado s.

        Modelo do sensor: vetor one-hot do estado real somado a ruído
        gaussiano de variância R, limitado ao intervalo [0, 1].
        """
        sigma = self.sensor_variance ** 0.5
        z = []
        for s in range(self.n_states):
            ideal = 1.0 if s == self._true_state else 0.0
            z.append(min(1.0, max(0.0, ideal + self._rng.gauss(0.0, sigma))))
        return z


# ============================================================================
# OBSTÁCULO MÓVEL (evento dinâmico)
# ============================================================================
class MovingObstacle:
    """Obstáculo móvel que caminha aleatoriamente pela grade (evento dinâmico).

    Ao ocupar uma célula da rota do drone, força o replanejamento do A*.
    """

    def __init__(self, cell: Cell) -> None:
        self.cell = cell

    def move(self, env: "Environment", forbidden: set[Cell],
             rng: random.Random) -> None:
        """Move para uma vizinha livre que não esteja em ``forbidden``."""
        options = [c for c in env.neighbors(self.cell)
                   if not env.is_wall(c) and c not in forbidden]
        if options:
            self.cell = rng.choice(options)


# ============================================================================
# AMBIENTE (mapa como grafo + eventos + sensores)
# ============================================================================
class Environment:
    """O mundo do jogo: grade/grafo, eventos estáticos e dinâmicos, sensores.

    Eventos estáticos (paredes, zonas perigosas) entram direto no grafo e NÃO
    passam por Kalman/Markov (são determinísticos). Eventos dinâmicos (as
    condições adversas e os obstáculos móveis) mudam a cada ``tick``.
    """

    def __init__(self, layout: list[str], seed: int | None = None) -> None:
        self.rng = random.Random(seed)
        self.rows = len(layout)
        self.cols = len(layout[0])
        assert all(len(row) == self.cols for row in layout), "mapa irregular"

        self.walls: set[Cell] = set()
        self.risk: set[Cell] = set()
        self.clients: list[Cell] = []
        self.restaurant: Cell = (0, 0)
        for y, row in enumerate(layout):
            for x, ch in enumerate(row):
                if ch == "#":
                    self.walls.add((x, y))
                elif ch == "~":
                    self.risk.add((x, y))
                elif ch == "R":
                    self.restaurant = (x, y)
                elif ch == "C":
                    self.clients.append((x, y))

        self.conditions = [
            AdverseCondition(c["name"], c["labels"], c["penalties"],
                             c["transition"], Config.SENSOR_VARIANCE, self.rng)
            for c in Config.CONDITIONS
        ]
        self.weather_zones = [
            {(x, y) for y in range(y_min, min(y_max, self.rows))
             for x in range(x_min, min(x_max, self.cols))}
            for x_min, y_min, x_max, y_max in Config.WEATHER_ZONES
        ]
        self.obstacles = self._spawn_obstacles(Config.N_OBSTACLES)

    # --- consultas do grafo -------------------------------------------------
    def in_bounds(self, cell: Cell) -> bool:
        """True se a célula está dentro da grade."""
        return 0 <= cell[0] < self.cols and 0 <= cell[1] < self.rows

    def is_wall(self, cell: Cell) -> bool:
        """True se a célula é um prédio fixo (aresta removida do grafo)."""
        return cell in self.walls

    def neighbors(self, cell: Cell) -> list[Cell]:
        """Vizinhas nas 4 direções (dentro da grade, ainda sem filtrar bloqueios)."""
        x, y = cell
        cand = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [c for c in cand if self.in_bounds(c)]

    def step_cost(self, cell: Cell) -> float:
        """Custo estático de entrar numa célula: 1, ou a penalidade da zona perigosa."""
        return Config.RISK_PENALTY if cell in self.risk else 1.0

    def blocked_cells(self) -> set[Cell]:
        """Todas as células intransitáveis agora: prédios + obstáculos móveis."""
        return self.walls | {o.cell for o in self.obstacles}

    def toggle_wall(self, cell: Cell) -> None:
        """Cria/remove um prédio (evento estático) - exceto em restaurante/clientes."""
        if cell == self.restaurant or cell in self.clients:
            return
        if cell in self.walls:
            self.walls.discard(cell)
        else:
            self.walls.add(cell)
            self.risk.discard(cell)

    # --- dinâmica -----------------------------------------------------------
    def tick(self, protected: set[Cell]) -> None:
        """Avança o ambiente um passo: clima (Markov) e obstáculos móveis.

        ``protected``: células onde os obstáculos não podem entrar (ex.: o drone).
        """
        for cond in self.conditions:
            cond.step()
        for obs in self.obstacles:
            if self.rng.random() < Config.OBSTACLE_MOVE_PROB:
                others = {o.cell for o in self.obstacles if o is not obs}
                obs.move(self, protected | others | {self.restaurant}
                         | set(self.clients), self.rng)

    def measure(self) -> list[list[float]]:
        """Lê os sensores de todas as condições: uma medida z_k para cada k."""
        return [cond.sense() for cond in self.conditions]

    def _spawn_obstacles(self, n: int) -> list[MovingObstacle]:
        free = [(x, y) for y in range(self.rows) for x in range(self.cols)
                if (x, y) not in self.walls and (x, y) != self.restaurant
                and (x, y) not in self.clients]
        far = [c for c in free if abs(c[0]) + abs(c[1]) > 8]
        return [MovingObstacle(c) for c in self.rng.sample(far, n)]


# ============================================================================
# AGENTE (o drone)
# ============================================================================
class Agent:
    """O drone entregador: percebe, estima, prevê, planeja e age.

    A cada atualização do ambiente, ``perceive`` executa o pipeline do documento:

        1. ``kalman_filter``  limpa a medida ruidosa z usando a previsão de Markov
        2. ``markov_chain``   projeta as probabilidades Δ passos à frente
        3. ``compute_costs``  converte probabilidades em C_pres e C_fut
        4. ``a_star``         busca a rota de menor  f(n) = C_pres·g(n) + C_fut·h(n)

    Se |C_pres - C_pres(t0)| > τ, ou a rota ficar bloqueada, o A* roda de novo
    a partir da posição atual (política de replanejamento da seção 5.6).

    Convenção de índices (mesma do documento):
        k = condição adversa (0..K-1)     s = estado da condição (0..m-1)
        self.pi[k][s]  = π_k,s  (estimativa corrigida)
        self.P[k][s]   = P_k,s  (incerteza da estimativa no Kalman)
    """

    def __init__(self, env: Environment, start: Cell) -> None:
        self.env = env
        self.position: Cell = start
        self.goal: Cell | None = None
        self.path: list[Cell] = []          # células restantes até o destino

        # Crenças iniciais: assume-se tudo "normal" com incerteza máxima (P=1).
        self.pi = [[1.0] + [0.0] * (c.n_states - 1) for c in env.conditions]
        self.P = [[1.0] * c.n_states for c in env.conditions]

        self.c_pres = 1.0                   # custo presente C_pres
        self.c_fut = 1.0                    # custo futuro   C_fut
        self.c_pres_at_plan = 1.0           # C_pres no último planejamento (t0)
        self.weather_cost_at_plan = 0.0

        self.replans = 0                    # nº de replanejamentos (estatística)
        self.last_reason = "-"              # motivo do último replanejamento
        self.expanded = 0                   # nós expandidos no último A*

    # ------------------------------------------------------------------
    # Utilitários matemáticos
    # ------------------------------------------------------------------
    @staticmethod
    def _vec_mat(v: list[float], matrix: list[list[float]]) -> list[float]:
        """Produto vetor-linha × matriz:  (v·M)_j = Σ_i v_i · M[i][j]."""
        return [sum(v[i] * matrix[i][j] for i in range(len(v)))
                for j in range(len(matrix[0]))]

    @staticmethod
    def _manhattan(a: Cell, b: Cell) -> int:
        """Heurística h(n): distância de Manhattan |x_n-x_B| + |y_n-y_B|."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def weather_step_cost(self, cell: Cell) -> float:
        """Penalidade climática esperada ao entrar na zona da célula."""
        penalty = 0.0
        for k, zone in enumerate(self.env.weather_zones):
            if cell in zone:
                penalty += sum(prob * (level - 1.0)
                               for prob, level in zip(self.pi[k],
                                                      self.env.conditions[k].penalties))
        return penalty

    def route_weather_cost(self, path: list[Cell]) -> float:
        """Soma as penalidades climáticas esperadas da rota atual."""
        return sum(self.weather_step_cost(cell) for cell in path)

    # ------------------------------------------------------------------
    # MÉTODO 1: Cadeia de Markov (seção 5.4)
    # ------------------------------------------------------------------
    def markov_chain(self, k: int, pi: list[float], steps: int) -> list[float]:
        """Projeta a distribuição ``pi`` da condição ``k`` para ``steps`` passos à frente.

            π(t+1) = π(t) · T_k        π(t+Δ) = π(t) · T_k^Δ

        O próximo estado depende só do estado atual (propriedade de Markov),
        então basta multiplicar o vetor pela matriz de transição ``steps`` vezes.

        Args:
            k: índice da condição adversa.
            pi: distribuição de probabilidade atual (soma 1).
            steps: quantos passos projetar (Δ). Com 1, é a previsão do Kalman.

        Returns:
            Nova distribuição de probabilidade (soma 1).
        """
        transition = self.env.conditions[k].transition
        for _ in range(steps):
            pi = self._vec_mat(pi, transition)
        return pi

    # ------------------------------------------------------------------
    # MÉTODO 2: Filtro de Kalman (seção 5.3)
    # ------------------------------------------------------------------
    def kalman_filter(self, k: int, z: list[float]) -> list[float]:
        """Funde a previsão de Markov com a medida ruidosa ``z`` da condição ``k``.

        Para cada estado s executa os três passos do documento:

            1. Previsão:  π⁻ = π(t-1)·T_k          P⁻ = P(t-1) + Q
            2. Ganho:     K  = P⁻ / (P⁻ + R)
            3. Correção:  π  = π⁻ + K·(z - π⁻)     P  = (1 - K)·P⁻

        Sensor ruidoso (R grande) => K perto de 0 => confia no modelo (Markov).
        Sensor bom (R pequeno)    => K perto de 1 => confia na medida.
        Por fim, a estimativa é normalizada (valores negativos viram 0 e a soma
        volta a ser 1) para continuar sendo uma distribuição de probabilidade.

        Args:
            k: índice da condição adversa.
            z: medida do sensor para cada estado (z_k,s).

        Returns:
            Estimativa corrigida π_k (também guarda P_k atualizado em ``self.P``).
        """
        r = self.env.conditions[k].sensor_variance
        predicted = self.markov_chain(k, self.pi[k], 1)         # passo 1 (π⁻)

        estimate, new_p = [], []
        for s, pred in enumerate(predicted):
            p_pred = self.P[k][s] + Config.Q                     # passo 1 (P⁻)
            gain = p_pred / (p_pred + r)                         # passo 2
            estimate.append(pred + gain * (z[s] - pred))         # passo 3
            new_p.append((1.0 - gain) * p_pred)
        self.P[k] = new_p

        clipped = [max(0.0, v) for v in estimate]                # normalização
        total = sum(clipped)
        return [v / total for v in clipped] if total > 0 else predicted

    # ------------------------------------------------------------------
    # Pesos adaptativos e custos (seção 5.5)
    # ------------------------------------------------------------------
    def compute_costs(self) -> None:
        """Converte as probabilidades em C_pres e C_fut.

            w_k = 1 - π_k,0                  (prob. de estar em estado adverso)
            c_k = w_k / Σ_j w_j              (normalizado; se Σw = 0, c_k = 1/K)
            C_pres = Σ_k c_k · Σ_s π_k,s · ℓ_k,s
            C_fut  = Σ_k c_k · Σ_s (π_k·T_k^Δ)_s · ℓ_k,s

        Como ℓ ≥ 1, ambos valem ≥ 1 (=1 quando tudo está normal).
        C_fut > C_pres significa que o ambiente tende a piorar.
        """
        conds = self.env.conditions
        weights = [1.0 - self.pi[k][0] for k in range(len(conds))]
        total = sum(weights)
        coefs = ([w / total for w in weights] if total > 1e-9
                 else [1.0 / len(conds)] * len(conds))

        self.c_pres = self.c_fut = 0.0
        for k, cond in enumerate(conds):
            future = self.markov_chain(k, self.pi[k], Config.DELTA)
            for s, pen in enumerate(cond.penalties):
                self.c_pres += coefs[k] * self.pi[k][s] * pen
                self.c_fut += coefs[k] * future[s] * pen

    # ------------------------------------------------------------------
    # MÉTODO 3: Algoritmo A* (seção 5.6)
    # ------------------------------------------------------------------
    def a_star(self, start: Cell, goal: Cell) -> list[Cell] | None:
        """Busca a rota de menor custo entre ``start`` e ``goal`` na grade.

        Função de avaliação (seção 5.6):

            f(n) = C_pres · g(n) + C_fut · h(n)

        onde g(n) inclui o custo estático da célula, o risco fixo e a penalidade
        climática esperada na zona local; h(n) é a distância de Manhattan até o
        destino, ponderada por C_fut. Células bloqueadas ficam fora do grafo.

        Observação: como os pesos C_pres e C_fut podem diferir, a heurística
        não é garantidamente consistente; por isso o código permite reabrir
        nós quando encontra um caminho melhor (g menor), o que preserva a
        correção do algoritmo.

        Returns:
            Lista de células do próximo passo até o destino (sem incluir
            ``start``), lista vazia se já está no destino, ou ``None`` se não
            existe rota.
        """
        if start == goal:
            return []
        blocked = self.env.blocked_cells()
        if goal in blocked:
            return None

        tie = itertools.count()                   # desempate estável no heap
        g_cost: dict[Cell, float] = {start: 0.0}
        came_from: dict[Cell, Cell] = {}
        f0 = self.c_fut * self._manhattan(start, goal)
        open_heap = [(f0, next(tie), 0.0, start)]  # (f, desempate, g, célula)
        self.expanded = 0

        while open_heap:
            _, _, g_node, node = heapq.heappop(open_heap)
            if g_node > g_cost.get(node, float("inf")):
                continue                          # entrada velha do heap
            if node == goal:
                return self._reconstruct(came_from, node)
            self.expanded += 1

            for nb in self.env.neighbors(node):
                if nb in blocked:
                    continue
                g_new = g_node + self.env.step_cost(nb) + self.weather_step_cost(nb)
                if g_new < g_cost.get(nb, float("inf")):
                    g_cost[nb] = g_new
                    came_from[nb] = node
                    f_new = (self.c_pres * g_new
                             + self.c_fut * self._manhattan(nb, goal))
                    heapq.heappush(open_heap, (f_new, next(tie), g_new, nb))
        return None

    @staticmethod
    def _reconstruct(came_from: dict[Cell, Cell], node: Cell) -> list[Cell]:
        """Refaz o caminho seguindo os predecessores do destino até a origem."""
        path = []
        while node in came_from:
            path.append(node)
            node = came_from[node]
        path.reverse()
        return path

    # ------------------------------------------------------------------
    # Decisão e ação
    # ------------------------------------------------------------------
    def set_goal(self, goal: Cell) -> None:
        """Define um novo destino e planeja a primeira rota (não conta como replan)."""
        self.goal = goal
        self.plan()

    def plan(self) -> None:
        """Executa o A* da posição atual até o destino e guarda a rota."""
        if self.goal is None:
            return
        route = self.a_star(self.position, self.goal)
        self.path = route if route is not None else []
        self.c_pres_at_plan = self.c_pres     # guarda C_pres(t0) para o limiar τ
        self.weather_cost_at_plan = self.route_weather_cost(self.path)

    def replan(self, reason: str) -> None:
        """Replaneja registrando o motivo e contabilizando a estatística."""
        self.plan()
        self.replans += 1
        self.last_reason = reason

    def _replan_reason(self) -> str | None:
        """Aplica a política de replanejamento; devolve o motivo ou ``None``."""
        if self.goal is None or self.position == self.goal:
            return None
        if not self.path:
            return "sem rota"
        blocked = self.env.blocked_cells()
        if any(cell in blocked for cell in self.path):
            return "rota bloqueada"
        if abs(self.route_weather_cost(self.path) - self.weather_cost_at_plan) > Config.TAU:
            return "custo climático da rota alterado"
        if abs(self.c_pres - self.c_pres_at_plan) > Config.TAU:
            return "variação de C_pres > τ"
        return None

    def perceive(self, measurements: list[list[float]]) -> None:
        """Executa o pipeline completo com as novas medidas dos sensores.

        Kalman (com Markov) -> custos -> decide se precisa replanejar com A*.
        """
        for k, z in enumerate(measurements):
            self.pi[k] = self.kalman_filter(k, z)
        self.compute_costs()
        reason = self._replan_reason()
        if reason:
            self.replan(reason)

    def move(self) -> None:
        """Avança o drone uma célula na rota; espera se o caminho estiver bloqueado."""
        reason = self._replan_reason()
        if reason in ("sem rota", "rota bloqueada"):
            self.replan(reason)
        if not self.path or self.path[0] in self.env.blocked_cells():
            return                            # sem caminho livre: aguarda
        self.position = self.path.pop(0)

    @property
    def arrived(self) -> bool:
        """True se o drone está sobre o destino."""
        return self.goal is not None and self.position == self.goal

    @property
    def trend(self) -> str:
        """Tendência do ambiente: compara custo futuro com custo presente."""
        if self.c_fut > self.c_pres + 0.02:
            return "piorando"
        if self.c_fut < self.c_pres - 0.02:
            return "melhorando"
        return "estável"


# ============================================================================
# RENDERIZAÇÃO
# ============================================================================
class Renderer:
    """Desenha o mapa, o drone e o painel de informações. Não altera o estado."""

    def __init__(self, screen: pygame.Surface, env: Environment) -> None:
        self.screen = screen
        self.env = env
        self.font_s = pygame.font.Font(None, 20)
        self.font_m = pygame.font.Font(None, 24)
        self.font_l = pygame.font.Font(None, 34)
        self.grid_w = env.cols * Config.CELL
        self.grid_h = env.rows * Config.CELL

    def _center(self, cell: Cell) -> tuple[int, int]:
        """Centro em pixels de uma célula."""
        c = Config.CELL
        return cell[0] * c + c // 2, cell[1] * c + c // 2

    def _text(self, txt: str, pos: tuple[int, int], font: pygame.font.Font,
              color: tuple[int, int, int] = Config.TEXT) -> None:
        self.screen.blit(font.render(txt, True, color), pos)

    def draw(self, game: "Game") -> None:
        """Desenha um quadro completo."""
        self.screen.fill(Config.BG)
        self._draw_map(game)
        self._draw_route(game.agent)
        self._draw_entities(game)
        self._draw_weather_tint(game.agent)
        self._draw_panel(game)
        pygame.display.flip()

    # --- mapa -----------------------------------------------------------------
    def _draw_map(self, game: "Game") -> None:
        c = Config.CELL
        for y in range(self.env.rows):
            for x in range(self.env.cols):
                cell = (x, y)
                if cell in self.env.walls:
                    color = Config.WALL
                elif cell in self.env.risk:
                    color = Config.RISK
                else:
                    color = Config.FREE
                rect = pygame.Rect(x * c, y * c, c, c)
                pygame.draw.rect(self.screen, color, rect)
                if cell not in self.env.walls:
                    for k, zone in enumerate(self.env.weather_zones):
                        if cell in zone:
                            tint = Config.WEATHER_COLORS[k]
                            blended = tuple((base * 2 + overlay) // 3
                                            for base, overlay in zip(color, tint))
                            pygame.draw.rect(self.screen, blended, rect)
                            break
                pygame.draw.rect(self.screen, Config.GRID, rect, 1)

    def _draw_route(self, agent: Agent) -> None:
        if not agent.path:
            return
        points = [self._center(agent.position)] + [self._center(c) for c in agent.path]
        pygame.draw.lines(self.screen, Config.ROUTE, False, points, 3)

    def _draw_entities(self, game: "Game") -> None:
        c = Config.CELL
        # restaurante e clientes
        rx, ry = self._center(self.env.restaurant)
        pygame.draw.rect(self.screen, Config.RESTAURANT,
                         pygame.Rect(rx - 14, ry - 14, 28, 28), border_radius=6)
        self._text("R", (rx - 5, ry - 7), self.font_m, Config.BG)
        for client in self.env.clients:
            cx, cy = self._center(client)
            is_goal = client == game.agent.goal
            pygame.draw.circle(self.screen, Config.CLIENT, (cx, cy), 14)
            if is_goal:
                pygame.draw.circle(self.screen, Config.TEXT, (cx, cy), 17, 2)
            self._text("C", (cx - 5, cy - 7), self.font_m, Config.BG)
        # obstáculos móveis
        for obs in self.env.obstacles:
            pygame.draw.circle(self.screen, Config.OBSTACLE,
                               self._center(obs.cell), c // 3)
        # drone (com "pacote" quando está levando um pedido)
        dx, dy = self._center(game.agent.position)
        pygame.draw.circle(self.screen, Config.DRONE, (dx, dy), 11)
        pygame.draw.circle(self.screen, Config.BG, (dx, dy), 11, 2)
        if game.carrying:
            pygame.draw.rect(self.screen, Config.RESTAURANT,
                             pygame.Rect(dx - 5, dy - 5, 10, 10))

    def _draw_weather_tint(self, agent: Agent) -> None:
        """Escurece o mapa proporcionalmente ao custo presente (clima pior)."""
        alpha = min(110, int((agent.c_pres - 1.0) * 140))
        if alpha <= 0:
            return
        tint = pygame.Surface((self.grid_w, self.grid_h), pygame.SRCALPHA)
        tint.fill((40, 70, 140, alpha))
        self.screen.blit(tint, (0, 0))

    # --- painel lateral ---------------------------------------------------------
    def _draw_panel(self, game: "Game") -> None:
        x0 = self.grid_w + 16
        y = 10
        self._text("AeroDelivery", (x0, y), self.font_l)
        y += 34
        self._text("Estimativa do drone (Kalman + Markov)", (x0, y),
                   self.font_s, Config.MUTED)
        y += 22

        for k, cond in enumerate(self.env.conditions):
            self._text(cond.name, (x0, y), self.font_m)
            y += 20
            for s, label in enumerate(cond.labels):
                prob = game.agent.pi[k][s]
                if game.show_truth and cond.true_state == s:
                    pygame.draw.circle(self.screen, Config.TRUTH, (x0 + 5, y + 7), 4)
                self._text(label, (x0 + 16, y), self.font_s, Config.MUTED)
                bar = pygame.Rect(x0 + 104, y + 2, 150, 12)
                pygame.draw.rect(self.screen, Config.BAR_BG, bar)
                pygame.draw.rect(self.screen, Config.BAR,
                                 (bar.x, bar.y, int(bar.w * prob), bar.h))
                self._text(f"{prob:4.2f}", (bar.right + 8, y), self.font_s)
                y += 16
            y += 6

        a = game.agent
        self._text(f"C_pres = {a.c_pres:.3f}", (x0, y), self.font_m)
        self._text(f"C_fut  = {a.c_fut:.3f}  ({a.trend})", (x0, y + 20), self.font_m)
        y += 50
        for line in (f"Entregas: {game.deliveries}",
                     f"Replanejamentos: {a.replans}  (τ = {Config.TAU})",
                     f"Último motivo: {a.last_reason}",
                     f"Nós expandidos no A*: {a.expanded}",
                     f"Missão: {'entregar pedido' if game.carrying else 'voltar ao restaurante'}"):
            self._text(line, (x0, y), self.font_s)
            y += 17
        y += 8
        legend = ["ESPAÇO pausa | R replanejar | T estado real (●)",
                  "CLIQUE cria/remove prédio | ESC sai",
                  "Clima: vento azul | chuva verde | neblina cinza"]
        if game.paused:
            legend.insert(0, ">> PAUSADO <<")
        for line in legend:
            self._text(line, (x0, y), self.font_s, Config.MUTED)
            y += 16


# ============================================================================
# JOGO (laço principal)
# ============================================================================
class Game:
    """Orquestra ambiente, agente, entrada do usuário e o despacho de pedidos."""

    def __init__(self, seed: int | None = None, headless: bool = False) -> None:
        pygame.init()
        self.env = Environment(Config.MAP, seed)
        width = self.env.cols * Config.CELL + Config.PANEL_W
        height = self.env.rows * Config.CELL
        pygame.display.set_caption("AeroDelivery - protótipo")
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.renderer = Renderer(self.screen, self.env)

        self.agent = Agent(self.env, self.env.restaurant)
        self.deliveries = 0
        self.carrying = True                       # começa com um pedido em mãos
        self.paused = False
        self.show_truth = True
        self.running = True
        self._env_timer = 0.0
        self._move_timer = 0.0
        self.agent.set_goal(self.env.rng.choice(self.env.clients))

    # --- lógica -----------------------------------------------------------------
    def update(self, dt: float) -> None:
        """Avança a simulação ``dt`` segundos (separado do laço para poder testar)."""
        if self.paused:
            return
        self._env_timer += dt
        self._move_timer += dt

        if self._env_timer >= Config.ENV_TICK:
            self._env_timer -= Config.ENV_TICK
            self.env.tick(protected={self.agent.position})
            self.agent.perceive(self.env.measure())          # pipeline completo

        if self._move_timer >= Config.MOVE_INTERVAL:
            self._move_timer -= Config.MOVE_INTERVAL
            self.agent.move()
            if self.agent.arrived:
                self._dispatch_next()

    def _dispatch_next(self) -> None:
        """Ao chegar: entrega o pedido e volta ao restaurante, ou pega um novo pedido."""
        if self.carrying:
            self.deliveries += 1
            self.carrying = False
            self.agent.set_goal(self.env.restaurant)
        else:
            self.carrying = True
            self.agent.set_goal(self.env.rng.choice(self.env.clients))

    # --- entrada ----------------------------------------------------------------
    def _handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_t:
                    self.show_truth = not self.show_truth
                elif event.key == pygame.K_r:
                    self.agent.replan("manual (tecla R)")
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                cell = (event.pos[0] // Config.CELL, event.pos[1] // Config.CELL)
                if self.env.in_bounds(cell) and cell != self.agent.position:
                    self.env.toggle_wall(cell)               # evento estático
                    self.agent.replan("mapa alterado")

    def run(self) -> None:
        """Laço principal: eventos -> atualização -> desenho."""
        while self.running:
            dt = self.clock.tick(Config.FPS) / 1000.0
            self._handle_events()
            self.update(dt)
            self.renderer.draw(self)
        pygame.quit()


if __name__ == "__main__":
    Game().run()
    sys.exit(0)
