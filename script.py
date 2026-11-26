"""
Roteamento dinamico com A* para o Pioneer P3DX no CoppeliaSim.

Cenario: robo sai do ponto A e vai ate o ponto B. Nove cancelas (teclas 1..9)
podem ser baixadas/levantadas a qualquer momento. O robo mantem um mapa
(grid de ocupacao) e replaneja a rota com A* sempre que o mapa muda.
"""
import csv
import heapq
import math
import time

import keyboard
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Limites da arena (metros)
X_MIN, X_MAX = -6.40, 6.40
Y_MIN, Y_MAX = -7.56364, 7.56364

TAM_CELULA = 0.1123456891234578912  # resolucao do grid (m)
RAIO_ROBO = 0.22  # raio do Pioneer + folga (m)
RAIO_PERCEPCAO = 1.0  # distancia (m) em que o robo "enxerga" uma cancela

PREFIXOS_OBSTACULOS = ["Obstaculo", "Osbtaculo", "obstaculo", "Cuboid", "Parede"]

# Tamanho da regiao bloqueada por cada cancela fechada
MEIA_CANCELA_H = (0.45, 0.10)
MEIA_CANCELA_V = (0.10, 0.45)
CANCELAS_VERTICAIS = {9}

ANGULO_CANCELA_ABERTA = 0.0
ANGULO_CANCELA_FECHADA = 1.57

VELOCIDADE_BASE = 0.6  # rad/s nas rodas
GANHO_ANGULAR = 1.5
DIST_WAYPOINT = 0.20  # distancia para considerar waypoint atingido (m)
DIST_CHEGADA = 0.30  # distancia para considerar chegada em B (m)

# Parametros de deteccao de colisao fisica e marcha-re
DIST_COLISAO_US = 0.08      # distancia (m) nos sensores frontais (2..5) para acionar re
TEMPO_RECUO_MAX = 1.1       # segundos dando marcha-re
VEL_RE = 2.0                # rad/s nas rodas durante a re
JANELA_TRAVADO = 0.7        # janela de tempo (s) para verificar deslocamento fisico
DIST_MIN_MOVIMENTO = 0.025  # deslocamento minimo esperado (m) na janela
DURACAO_BLOQUEIO_TEMP = 6.0 # segundos que um ponto de colisao fica bloqueado no mapa

ARQ_REPLAN = "log_replanejamentos.csv"
ARQ_RESUMO = "resumo_missao.csv"

# =====================================================================
# GRID DE OCUPACAO
# =====================================================================
NX = int(round((X_MAX - X_MIN) / TAM_CELULA))
NY = int(round((Y_MAX - Y_MIN) / TAM_CELULA))


def mundo_para_celula(x, y):
    i = int((x - X_MIN) / TAM_CELULA)
    j = int((y - Y_MIN) / TAM_CELULA)
    return (min(max(i, 0), NX - 1), min(max(j, 0), NY - 1))


def celula_para_mundo(c):
    return (X_MIN + (c[0] + 0.5) * TAM_CELULA, Y_MIN + (c[1] + 0.5) * TAM_CELULA)


def centros_celulas():
    xs = X_MIN + (np.arange(NX) + 0.5) * TAM_CELULA
    ys = Y_MIN + (np.arange(NY) + 0.5) * TAM_CELULA
    return np.meshgrid(xs, ys, indexing="ij")


def inflar(grid, raio_m):
    r = int(math.ceil(raio_m / TAM_CELULA))
    saida = grid.copy()
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj > r * r:
                continue
            deslocado = np.zeros_like(grid)
            i0, i1 = max(di, 0), NX + min(di, 0)
            j0, j1 = max(dj, 0), NY + min(dj, 0)
            deslocado[i0:i1, j0:j1] = grid[i0 - di:i1 - di, j0 - dj:j1 - dj]
            saida |= deslocado
    return saida


def mascara_retangulo_orientado(sim, handle):
    mn = [sim.getObjectFloatParam(handle, p) for p in (
        sim.objfloatparam_objbbox_min_x, sim.objfloatparam_objbbox_min_y,
        sim.objfloatparam_objbbox_min_z)]
    mx = [sim.getObjectFloatParam(handle, p) for p in (
        sim.objfloatparam_objbbox_max_x, sim.objfloatparam_objbbox_max_y,
        sim.objfloatparam_objbbox_max_z)]
    m = sim.getObjectMatrix(handle, sim.handle_world)
    R = np.array([[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]])
    t = np.array([m[3], m[7], m[11]])
    gx, gy = centros_celulas()
    pts = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, t[2])], axis=1)
    local = (pts - t) @ R
    dentro = ((local[:, 0] >= mn[0]) & (local[:, 0] <= mx[0]) &
              (local[:, 1] >= mn[1]) & (local[:, 1] <= mx[1]))
    return dentro.reshape(NX, NY)


def mascara_cancela(centro_xy, meia):
    gx, gy = centros_celulas()
    return ((np.abs(gx - centro_xy[0]) <= meia[0]) &
            (np.abs(gy - centro_xy[1]) <= meia[1]))


# =====================================================================
# A*
# =====================================================================
VIZINHOS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def heuristica_octil(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


def a_estrela(grid, inicio, fim, usar_heuristica=True):
    t0 = time.perf_counter()
    abertos = [(0.0, inicio)]
    g = {inicio: 0.0}
    pai = {}
    fechados = set()
    expandidos = 0
    while abertos:
        _, atual = heapq.heappop(abertos)
        if atual in fechados:
            continue
        fechados.add(atual)
        expandidos += 1
        if atual == fim:
            caminho = [atual]
            while atual in pai:
                atual = pai[atual]
                caminho.append(atual)
            return caminho[::-1], expandidos, (time.perf_counter() - t0) * 1000
        for dx, dy, custo_mov in VIZINHOS:
            viz = (atual[0] + dx, atual[1] + dy)
            if not (0 <= viz[0] < NX and 0 <= viz[1] < NY) or grid[viz]:
                continue
            if dx and dy and (grid[atual[0] + dx, atual[1]] or grid[atual[0], atual[1] + dy]):
                continue
            novo_g = g[atual] + custo_mov
            if novo_g < g.get(viz, float("inf")):
                g[viz] = novo_g
                pai[viz] = atual
                h = heuristica_octil(viz, fim) if usar_heuristica else 0.0
                heapq.heappush(abertos, (novo_g + h, viz))
    return None, expandidos, (time.perf_counter() - t0) * 1000


def linha_livre(grid, a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dist = math.hypot(dx, dy)
    if dist == 0:
        return True
    px, py = -dy / dist, dx / dist
    passos = int(math.ceil(dist * 2))
    for k in range(passos + 1):
        t = k / passos
        cx = a[0] + dx * t
        cy = a[1] + dy * t
        for desloc in (-0.8, 0.0, 0.8):
            i = round(cx + px * desloc)
            j = round(cy + py * desloc)
            if not (0 <= i < NX and 0 <= j < NY) or grid[i, j]:
                return False
    return True


def suavizar(grid, caminho):
    if len(caminho) <= 2:
        return caminho
    saida = [caminho[0]]
    i = 0
    while i < len(caminho) - 1:
        j = len(caminho) - 1
        while j > i + 1 and not linha_livre(grid, caminho[i], caminho[j]):
            j -= 1
        saida.append(caminho[j])
        i = j
    return saida


def comprimento_m(caminho):
    return sum(math.dist(celula_para_mundo(caminho[k]), celula_para_mundo(caminho[k + 1]))
               for k in range(len(caminho) - 1))


# =====================================================================
# FUNCAO DE BUSCA ROBUSTA DE OBJETOS
# =====================================================================
def buscar_objeto(sim, nomes_possiveis):
    handles = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    for h in handles:
        alias = sim.getObjectAlias(h, 0)
        if alias in nomes_possiveis:
            return h
    raise Exception(f"CRITICO: Nenhum destes objetos foi encontrado na cena: {nomes_possiveis}")


# =====================================================================
# CONEXAO E OBJETOS
# =====================================================================
print("Conectando ao CoppeliaSim...")
client = RemoteAPIClient()
sim = client.getObject("sim")

try:
    carro = buscar_objeto(sim, ["PioneerP3DX", "Pioneer_p3dx"])

    try:
        ponto_b = buscar_objeto(sim, ["PontoB"])
    except Exception:
        ponto_b = buscar_objeto(sim, ["A"])

    motorEsq = buscar_objeto(sim, ["leftMotor", "Pioneer_p3dx_leftMotor", "PioneerP3DX_leftMotor"])
    motorDir = buscar_objeto(sim, ["rightMotor", "Pioneer_p3dx_rightMotor", "PioneerP3DX_rightMotor"])

    sensores_frente = []
    for idx in (2, 3, 4, 5):
        try:
            s = buscar_objeto(sim, [
                f"ultrasonicSensor[{idx}]",
                f"Pioneer_p3dx_ultrasonicSensor[{idx}]",
                f"PioneerP3DX_ultrasonicSensor[{idx}]"
            ])
            sensores_frente.append((idx, s))
        except Exception:
            pass

    if not sensores_frente:
        print("Aviso: Sensores ultrassonicos frontais nao encontrados.")
    else:
        print(f"{len(sensores_frente)} sensores ultrassonicos frontais carregados.")

    motores_cancela = {n: buscar_objeto(sim, [f"motor{n}"]) for n in range(1, 10)}

    try:
        alvo = buscar_objeto(sim, ["Alvo"])
    except Exception:
        alvo = None
        print("Aviso: Objeto 'Alvo' nao encontrado. O robo navegara sem o marcador visual.")

except Exception as e:
    print(f"\nERRO: {e}")
    raise SystemExit(1)

# ---- mapa estatico: blocos brancos ----------------------------------
grid_fixo = np.zeros((NX, NY), dtype=bool)
blocos = []
for h in sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0):
    alias = sim.getObjectAlias(h, 0)
    if any(alias.startswith(p) for p in PREFIXOS_OBSTACULOS):
        if "Cancela" not in alias:
            blocos.append(h)
            grid_fixo |= mascara_retangulo_orientado(sim, h)

if not blocos:
    print("AVISO: nenhum obstaculo fixo encontrado. Verifique os nomes na hierarquia.")
print(f"{len(blocos)} blocos fixos lidos para o grid ({NX}x{NY} celulas de {TAM_CELULA:.3f} m).")

grid_fixo[0, :] = grid_fixo[-1, :] = True
grid_fixo[:, 0] = grid_fixo[:, -1] = True
grid_fixo_inflado = inflar(grid_fixo, RAIO_ROBO)

# ---- mascaras de cada cancela (inflada) ----------------------------
pos_vao = {}
for n in range(1, 10):
    pos_motor = sim.getObjectPosition(motores_cancela[n], sim.handle_world)
    if n in CANCELAS_VERTICAIS:
        pos_vao[n] = [pos_motor[0], pos_motor[1] + 0.45, pos_motor[2]]
    else:
        pos_vao[n] = [pos_motor[0] + 0.45, pos_motor[1], pos_motor[2]]

mascara_cancelas = {}
for n in range(1, 10):
    meia = MEIA_CANCELA_V if n in CANCELAS_VERTICAIS else MEIA_CANCELA_H
    mascara_cancelas[n] = inflar(mascara_cancela(pos_vao[n], meia), RAIO_ROBO)

# ---- estado ------------------------------------------------------
cancela_fechada = {n: False for n in range(1, 10)}
conhecido_fechada = {n: False for n in range(1, 10)}
tecla_anterior = {n: False for n in range(1, 10)}

caminho_mundo = []
idx_wp = 0
precisa_replanejar = True
sem_caminho = False
dist_percorrida = 0.0
n_replans = 0
pos_anterior = None
resultados_replan = []

# Variaveis de controle da marcha-re e bloqueios temporarios de quina
tempo_recuo_ate = 0.0
lado_re = 1
pos_check_travado = None
tempo_check_travado = 0.0
bloqueios_temp = []  # lista de (celula_i, celula_j, expira_em)

print("=" * 60)
print("TECLAS 1..9: baixa/sobe a cancela correspondente")
print("O robo vai de A ate B replanejando com A* quando o mapa muda")
print("=" * 60)

client.setStepping(True)
sim.startSimulation()
t_inicio_sim = sim.getSimulationTime()
chegou = False

try:
    while True:
        tempo_atual = sim.getSimulationTime()

        # ---------------- teclado -> cancelas ----------------
        for n in range(1, 10):
            atual = keyboard.is_pressed(str(n))
            if atual and not tecla_anterior[n]:
                cancela_fechada[n] = not cancela_fechada[n]
                print(f"\n[CIDADE] Cancela {n}: "
                      + ("FECHADA" if cancela_fechada[n] else "ABERTA"))
            tecla_anterior[n] = atual
            sim.setJointTargetPosition(
                motores_cancela[n],
                ANGULO_CANCELA_FECHADA if cancela_fechada[n] else ANGULO_CANCELA_ABERTA)

        # ---------------- pose do robo ----------------
        pos = sim.getObjectPosition(carro, sim.handle_world)
        ori = sim.getObjectOrientation(carro, sim.handle_world)
        yaw = ori[2]
        if pos_anterior is not None:
            dist_percorrida += math.dist(pos[:2], pos_anterior[:2])
        pos_anterior = pos
        pos_b = sim.getObjectPosition(ponto_b, sim.handle_world)

        if math.dist(pos[:2], pos_b[:2]) < DIST_CHEGADA:
            chegou = True
            break

        # ---------------- percepcao local das cancelas ----------------
        mudou = []
        for n in range(1, 10):
            if math.dist(pos[:2], pos_vao[n][:2]) <= RAIO_PERCEPCAO:
                if conhecido_fechada[n] != cancela_fechada[n]:
                    conhecido_fechada[n] = cancela_fechada[n]
                    mudou.append(n)
        if mudou:
            precisa_replanejar = True
            evento = "cancelas_mudaram:" + ",".join(map(str, mudou))
            print(f"[PERCEPCAO] Mapa atualizado -> {evento}")
        elif sem_caminho:
            evento = "retentativa"
        else:
            evento = "inicial"

        # ---------------- grid atual ----------------
        grid = grid_fixo_inflado.copy()
        for n in range(1, 10):
            if conhecido_fechada[n]:
                grid |= mascara_cancelas[n]

        # Aplica bloqueios temporarios de quinas onde o robo encostou
        bloqueios_temp = [b for b in bloqueios_temp if b[2] > tempo_atual]
        for bi, bj, _ in bloqueios_temp:
            grid[max(bi - 1, 0):bi + 2, max(bj - 1, 0):bj + 2] = True

        # se o caminho atual ficou invalido
        if caminho_mundo and not precisa_replanejar:
            for wp in caminho_mundo[idx_wp:]:
                if grid[mundo_para_celula(*wp)]:
                    precisa_replanejar = True
                    evento = "caminho_bloqueado"
                    break

        # ---------------- replanejamento ----------------
        if precisa_replanejar or (sem_caminho and int(tempo_atual * 10) % 10 == 0):
            inicio = mundo_para_celula(pos[0], pos[1])
            fim = mundo_para_celula(pos_b[0], pos_b[1])
            g_plan = grid.copy()
            # Restaura a vizinhanca imediata de inicio e fim para nao prender o robo
            # quando ele inicia ou termina perto de uma parede
            for c in (inicio, fim):
                g_plan[max(c[0] - 2, 0):c[0] + 3, max(c[1] - 2, 0):c[1] + 3] = \
                    grid_fixo[max(c[0] - 2, 0):c[0] + 3, max(c[1] - 2, 0):c[1] + 3]

            cam, nos_a, t_a = a_estrela(g_plan, inicio, fim, True)
            _, nos_d, t_d = a_estrela(g_plan, inicio, fim, False)

            if cam is None:
                if not sem_caminho:
                    print("\n[A*] Beco sem saida! Iniciando manobra de recuo...")
                    tempo_recuo_ate = tempo_atual + TEMPO_RECUO_MAX
                    bloqueios_temp.clear()

                    for n_c in range(1, 10):
                        if math.dist(pos[:2], pos_vao[n_c][:2]) > RAIO_PERCEPCAO:
                            conhecido_fechada[n_c] = False

                sem_caminho = True
                caminho_mundo = []
            else:
                sem_caminho = False
                tempo_recuo_ate = 0.0
                cam_suave = suavizar(g_plan, cam)
                caminho_mundo = [celula_para_mundo(c) for c in cam_suave]
                idx_wp = 1 if len(caminho_mundo) > 1 else 0
                n_replans += 1
                comp = comprimento_m(cam)
                print(f"[A*] replan #{n_replans}: {comp:.2f} m, {len(cam_suave)} waypoints, "
                      f"{nos_a} nos (Dijkstra: {nos_d}), {t_a:.1f} ms")
                resultados_replan.append({
                    "replan": n_replans,
                    "tempo_sim_s": round(tempo_atual - t_inicio_sim, 2),
                    "evento": evento,
                    "cancelas_fechadas_conhecidas": ",".join(str(n) for n in conhecido_fechada if conhecido_fechada[n]),
                    "tempo_astar_ms": round(t_a, 3),
                    "nos_astar": nos_a,
                    "tempo_dijkstra_ms": round(t_d, 3),
                    "nos_dijkstra": nos_d,
                    "comprimento_m": round(comp, 3),
                    "waypoints": len(cam_suave),
                })
            precisa_replanejar = False

        # ---------------- leitura dos sensores frontais (2 ao 5) ----------------
        menor_dist_esq = 999.0
        menor_dist_dir = 999.0

        for idx, s in sensores_frente:
            res, dist, *_ = sim.readProximitySensor(s)
            if res > 0:
                if idx <= 3:
                    menor_dist_esq = min(menor_dist_esq, dist)
                else:
                    menor_dist_dir = min(menor_dist_dir, dist)

        menor_dist_frente = min(menor_dist_esq, menor_dist_dir)

        def acionar_re_e_marcar_quina(motivo, lado):
            global tempo_recuo_ate, lado_re, pos_check_travado
            print(f"\n{motivo}")
            tempo_recuo_ate = tempo_atual + TEMPO_RECUO_MAX
            lado_re = lado
            pos_check_travado = None
            # Marca o ponto ~25 cm a frente do robo para o A* desviar dessa quina
            fx = pos[0] + 0.25 * math.cos(yaw)
            fy = pos[1] + 0.25 * math.sin(yaw)
            ci, cj = mundo_para_celula(fx, fy)
            bloqueios_temp.append((ci, cj, tempo_atual + DURACAO_BLOQUEIO_TEMP))

        if menor_dist_frente < DIST_COLISAO_US and tempo_atual >= tempo_recuo_ate:
            acionar_re_e_marcar_quina(
                f"[COLISAO US] Encostou ({menor_dist_frente:.2f} m)! Dando re...",
                -1 if menor_dist_esq < menor_dist_dir else 1
            )

        # ---------------- seguimento de caminho com recuo (re) ----------------
        if tempo_atual < tempo_recuo_ate:
            pos_check_travado = None
            if lado_re > 0:
                sim.setJointTargetVelocity(motorEsq, -VEL_RE * 1.25)
                sim.setJointTargetVelocity(motorDir, -VEL_RE * 0.75)
            else:
                sim.setJointTargetVelocity(motorEsq, -VEL_RE * 0.75)
                sim.setJointTargetVelocity(motorDir, -VEL_RE * 1.25)

            if (tempo_recuo_ate - tempo_atual) < 0.15:
                precisa_replanejar = True
                evento = "desvio_quina"

        elif sem_caminho or not caminho_mundo:
            sim.setJointTargetVelocity(motorEsq, 0.0)
            sim.setJointTargetVelocity(motorDir, 0.0)
            pos_check_travado = None

        else:
            while idx_wp < len(caminho_mundo) - 1 and \
                    math.dist(pos[:2], caminho_mundo[idx_wp]) < DIST_WAYPOINT:
                idx_wp += 1
            wp = caminho_mundo[idx_wp]

            if alvo is not None:
                sim.setObjectPosition(alvo, sim.handle_world, [wp[0], wp[1], pos[2]])
                rel = sim.getObjectPosition(alvo, carro)
            else:
                inv_m = sim.buildMatrixQ(sim.getObjectPosition(carro, sim.handle_world),
                                         sim.getObjectQuaternion(carro, sim.handle_world))
                sim.invertMatrix(inv_m)
                rel = sim.multiplyVector(inv_m, [wp[0], wp[1], pos[2]])

            erro = math.atan2(rel[1], rel[0])
            if abs(erro) > 1.0:
                v = 0.0
            else:
                v = VELOCIDADE_BASE * max(0.2, math.cos(erro))

            sim.setJointTargetVelocity(motorEsq, v - erro * GANHO_ANGULAR)
            sim.setJointTargetVelocity(motorDir, v + erro * GANHO_ANGULAR)

            if v > 0.2:
                if pos_check_travado is None:
                    pos_check_travado = pos[:2]
                    tempo_check_travado = tempo_atual
                elif tempo_atual - tempo_check_travado >= JANELA_TRAVADO:
                    deslocamento = math.dist(pos[:2], pos_check_travado)
                    if deslocamento < DIST_MIN_MOVIMENTO:
                        acionar_re_e_marcar_quina(
                            "[CONTATO FISICO] Robo preso na parede! Dando re...",
                            1 if erro >= 0 else -1
                        )
                    else:
                        pos_check_travado = pos[:2]
                        tempo_check_travado = tempo_atual
            else:
                pos_check_travado = None

        client.step()

except KeyboardInterrupt:
    print("\nInterrompido pelo usuario.")

finally:
    try:
        sim.setJointTargetVelocity(motorEsq, 0.0)
        sim.setJointTargetVelocity(motorDir, 0.0)
        tempo_total = sim.getSimulationTime() - t_inicio_sim
        sim.stopSimulation()
    except Exception:
        tempo_total = 0.0

    if resultados_replan:
        with open(ARQ_REPLAN, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(resultados_replan[0].keys()))
            w.writeheader()
            w.writerows(resultados_replan)
    with open(ARQ_RESUMO, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["chegou", "tempo_missao_s", "distancia_percorrida_m", "replanejamentos"])
        w.writerow([chegou, round(tempo_total, 2), round(dist_percorrida, 3), n_replans])

    print("\n" + "=" * 60)
    print(f"Missao {'CONCLUIDA' if chegou else 'INTERROMPIDA'}")
    print(f"Tempo: {tempo_total:.1f} s | Percorrido: {dist_percorrida:.2f} m | "
          f"Replanejamentos: {n_replans}")
    print(f"Logs salvos em {ARQ_REPLAN} e {ARQ_RESUMO}")
    print("Simulacao encerrada.")