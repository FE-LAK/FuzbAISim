import asyncio
import json
import logging
import os
import sqlite3
import time
from contextlib import asynccontextmanager
from typing import List, Optional

import httpx
import uvicorn
from fastapi import FastAPI, HTTPException, Request, Query
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.base import BaseHTTPMiddleware
from pydantic import BaseModel

from competition_proxy import ProxyManager

# --- Configuration & Setup ---
LOG_FORMAT = "%(asctime)s | %(name)s | %(levelname)s | %(message)s"
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
logger = logging.getLogger("competition_manager")

DB_PATH = "results.db"
CONFIG_PATH = "config.json"
WWW_DIR = "www"

# --- IP Whitelist Middleware ---

class IPWhitelistMiddleware(BaseHTTPMiddleware):
    """Restrict management/control pages to whitelisted IPs.
    Visualization page (/) is public. API and management pages require whitelist."""

    # Paths that are always public (no IP check)
    PUBLIC_PREFIXES = (
        "/visualization",
        "/static/",
        "/api/tournament/bracket",
        "/api/tournament/state",
        "/api/results/standings",
        "/api/tournament/viz-state",
    )

    def __init__(self, app, allowed_ips: list[str] = None):
        super().__init__(app)
        self.allowed_ips = set(allowed_ips or [])

    async def dispatch(self, request: Request, call_next):
        path = request.url.path

        # Always allow public paths
        if path == "/" or any(path.startswith(p) for p in self.PUBLIC_PREFIXES):
            return await call_next(request)

        # If no whitelist configured, allow all
        if not self.allowed_ips:
            return await call_next(request)

        # Get client IP
        client_ip = request.client.host if request.client else "unknown"

        # Always allow localhost
        if client_ip in ("127.0.0.1", "::1", "localhost"):
            return await call_next(request)

        if client_ip not in self.allowed_ips:
            return JSONResponse(
                status_code=403,
                content={"detail": f"Access denied for IP {client_ip}"}
            )

        return await call_next(request)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # --- Startup ---
    init_db()
    load_config()
    # Initialize proxy for enabled teams
    for team in teams.values():
        if team.enabled:
            await proxy.add_team(team.id, team.proxy_port)
    logger.info("Competition Manager v2 started.")

    yield

    # --- Shutdown ---
    await proxy.stop_all()
    logger.info("Competition Manager v2 stopped.")

app = FastAPI(
    title="FuzbAI Competition Manager v2",
    lifespan=lifespan
)
proxy = ProxyManager()

# --- Models ---

class Team(BaseModel):
    id: int
    name: str
    proxy_port: int
    enabled: bool = True

class Simulator(BaseModel):
    id: int
    host: str
    port_red: int
    port_blue: int
    port_mgmt: int
    enabled: bool = True

class GameResult(BaseModel):
    id: Optional[int] = None
    red_team_id: int
    blue_team_id: int
    red_team_name: str
    blue_team_name: str
    score_red: int
    score_blue: int
    game_time: float
    timestamp: Optional[str] = None

class MatchPairing(BaseModel):
    sim_id: int
    red_team_id: int
    blue_team_id: int

class TournamentGameResult(BaseModel):
    team1_score: int
    team2_score: int
    game_time: float = 0.0

class TournamentPairUpdate(BaseModel):
    pair_id: int
    team1_id: Optional[int] = None
    team2_id: Optional[int] = None

class VizStateUpdate(BaseModel):
    view: Optional[str] = None          # "bracket", "match", "standings", "team", "title"
    match_id: Optional[int] = None      # which match to show
    round_name: Optional[str] = None    # which round to focus
    team_id: Optional[int] = None       # which team to show
    live: Optional[bool] = None         # live indicator
    message: Optional[str] = None       # overlay message

# --- State ---

teams: dict[int, Team] = {}
simulators: dict[int, Simulator] = {}
active_matches: dict[int, MatchPairing] = {} # sim_id -> pairing
allowed_ips: list[str] = []

# Visualization state (controlled by viz-control, read by visualization)
viz_state = {
    "view": "title",
    "match_id": None,
    "round_name": None,
    "team_id": None,
    "live": False,
    "message": "",
    "tiktak": 0,
}

# SSE subscribers for visualization
viz_subscribers: list[asyncio.Queue] = []

# --- Database Initialization ---

def init_db():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS game_results (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            red_team_id INTEGER NOT NULL,
            blue_team_id INTEGER NOT NULL,
            red_team_name TEXT,
            blue_team_name TEXT,
            score_red INTEGER NOT NULL,
            score_blue INTEGER NOT NULL,
            game_time REAL,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)

    # Tournament tables
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS tournament_config (
            id INTEGER PRIMARY KEY CHECK (id = 1),
            phase TEXT NOT NULL DEFAULT 'preparation',
            confirmed INTEGER NOT NULL DEFAULT 0,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS tournament_pairs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            round TEXT NOT NULL,
            match_index INTEGER NOT NULL,
            team1_id INTEGER,
            team2_id INTEGER,
            team1_name TEXT,
            team2_name TEXT,
            team1_seed INTEGER,
            team2_seed INTEGER,
            winner_id INTEGER,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS tournament_games (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            pair_id INTEGER NOT NULL,
            game_number INTEGER NOT NULL,
            team1_score INTEGER DEFAULT 0,
            team2_score INTEGER DEFAULT 0,
            game_time REAL DEFAULT 0,
            status TEXT NOT NULL DEFAULT 'pending',
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            FOREIGN KEY (pair_id) REFERENCES tournament_pairs(id)
        )
    """)

    # Initialize tournament config if not exists
    cursor.execute("INSERT OR IGNORE INTO tournament_config (id, phase) VALUES (1, 'preparation')")

    conn.commit()
    conn.close()

# --- Configuration Persistence ---

def load_config():
    global teams, simulators, allowed_ips
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r") as f:
                data = json.load(f)
                teams = {t["id"]: Team(**t) for t in data.get("teams", [])}
                simulators = {s["id"]: Simulator(**s) for s in data.get("simulators", [])}
                allowed_ips = data.get("allowed_ips", [])
                logger.info("Configuration loaded from %s", CONFIG_PATH)
        except Exception as e:
            logger.error("Failed to load config: %s", e)

def save_config():
    data = {
        "teams": [t.model_dump() for t in teams.values()],
        "simulators": [s.model_dump() for s in simulators.values()],
        "allowed_ips": allowed_ips,
    }
    try:
        with open(CONFIG_PATH, "w") as f:
            json.dump(data, f, indent=4)
        logger.info("Configuration saved to %s", CONFIG_PATH)
    except Exception as e:
        logger.error("Failed to save config: %s", e)


# --- API Endpoints: Teams ---

@app.get("/api/teams")
async def get_teams():
    team_list = []
    all_stats = proxy.get_all_stats()
    for tid, team in teams.items():
        t_data = team.model_dump()
        t_data["stats"] = all_stats.get(tid)
        team_list.append(t_data)
    return team_list

@app.post("/api/teams")
async def add_team(team: Team):
    if team.id in teams:
        raise HTTPException(status_code=400, detail="Team ID already exists")
    teams[team.id] = team
    if team.enabled:
        await proxy.add_team(team.id, team.proxy_port)
    save_config()
    return team

@app.delete("/api/teams/{team_id}")
async def remove_team(team_id: int):
    if team_id not in teams:
        raise HTTPException(status_code=404, detail="Team not found")
    await proxy.remove_team(team_id)
    del teams[team_id]
    save_config()
    return {"status": "ok"}

@app.post("/api/teams/{team_id}/enable")
async def enable_team(team_id: int, enabled: bool):
    if team_id not in teams:
        raise HTTPException(status_code=404, detail="Team not found")
    teams[team_id].enabled = enabled
    if enabled:
        await proxy.add_team(team_id, teams[team_id].proxy_port)
    else:
        await proxy.remove_team(team_id)
    save_config()
    return {"enabled": enabled}

# --- API Endpoints: Simulators ---

@app.get("/api/simulators")
async def get_simulators():
    return list(simulators.values())

@app.post("/api/simulators")
async def add_simulator(sim: Simulator):
    if sim.id in simulators:
        raise HTTPException(status_code=400, detail="Simulator ID already exists")
    simulators[sim.id] = sim
    save_config()
    return sim

@app.delete("/api/simulators/{sim_id}")
async def remove_simulator(sim_id: int):
    if sim_id not in simulators:
        raise HTTPException(status_code=404, detail="Simulator not found")
    del simulators[sim_id]
    if sim_id in active_matches:
        del active_matches[sim_id]
    save_config()
    return {"status": "ok"}

@app.get("/api/simulators/{sim_id}/status")
async def get_sim_status(sim_id: int):
    if sim_id not in simulators:
        raise HTTPException(status_code=404, detail="Simulator not found")
    sim = simulators[sim_id]
    url = f"http://{sim.host}:{sim.port_mgmt}/status"
    try:
        async with httpx.AsyncClient() as client:
            resp = await client.get(url, timeout=0.2)            
            return resp.json()
    except Exception as e:
        return {"status": "offline", "error": str(e)}

@app.post("/api/simulators/{sim_id}/control")
async def control_sim(sim_id: int, command: str):
    if sim_id not in simulators:
        raise HTTPException(status_code=404, detail="Simulator not found")
    sim = simulators[sim_id]
    url = f"http://{sim.host}:{sim.port_mgmt}/{command}"
    try:
        async with httpx.AsyncClient() as client:
            resp = await client.post(url, timeout=2.0)
            return resp.json()
    except Exception as e:
        raise HTTPException(status_code=502, detail=f"Simulator error: {e}")

@app.post("/api/simulators/{sim_id}/enable")
async def enable_sim(sim_id: int, enabled: bool):
    if sim_id not in simulators:
        raise HTTPException(status_code=404, detail="Simulator not found")
    simulators[sim_id].enabled = enabled
    save_config()
    return {"enabled": enabled}

# --- API Endpoints: Matches & Pairing ---

@app.post("/api/matches/assign")
async def assign_match(pairing: MatchPairing):
    if pairing.sim_id not in simulators:
        raise HTTPException(status_code=404, detail="Simulator not found")
    if pairing.red_team_id not in teams or pairing.blue_team_id not in teams:
        raise HTTPException(status_code=404, detail="Team not found")
    
    sim = simulators[pairing.sim_id]

    await proxy.drop_connections(pairing.red_team_id)
    await proxy.drop_connections(pairing.blue_team_id)

    proxy.set_route(pairing.red_team_id, sim.host, sim.port_red)
    proxy.set_route(pairing.blue_team_id, sim.host, sim.port_blue)
    
    try:
        async with httpx.AsyncClient() as client:
            await client.post(
                f"http://{sim.host}:{sim.port_mgmt}/teams",
                json={
                    "red": teams[pairing.red_team_id].name,
                    "blue": teams[pairing.blue_team_id].name
                },
                timeout=1.0
            )
    except Exception as e:
        logger.warning("Failed to notify simulator %d of team names: %s", pairing.sim_id, e)

    active_matches[pairing.sim_id] = pairing
    return {"status": "assigned"}

@app.post("/api/matches/save-results")
async def save_match_results(sim_id: int):
    if sim_id not in active_matches:
        raise HTTPException(status_code=400, detail="No active match on this simulator")
    
    pairing = active_matches[sim_id]
    sim = simulators[sim_id]
    
    status_url = f"http://{sim.host}:{sim.port_mgmt}/status"
    try:
        async with httpx.AsyncClient() as client:
            resp = await client.get(status_url, timeout=2.0)
        status = resp.json()
        score = status.get("score", [0, 0])
        game_time = status.get("gametime", 0)
    except Exception as e:
        logger.error("Failed to fetch final status for saving: %s", e)
        raise HTTPException(status_code=502, detail="Failed to fetch final status from simulator")

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        INSERT INTO game_results (red_team_id, blue_team_id, red_team_name, blue_team_name, score_red, score_blue, game_time)
        VALUES (?, ?, ?, ?, ?, ?, ?)
    """, (
        pairing.red_team_id, pairing.blue_team_id,
        teams[pairing.red_team_id].name, teams[pairing.blue_team_id].name,
        score[0], score[1], game_time
    ))
    conn.commit()
    conn.close()

    proxy.clear_route(pairing.red_team_id)
    proxy.clear_route(pairing.blue_team_id)
    
    del active_matches[sim_id]
    return {"status": "saved"}

@app.get("/api/matches/active-count")
async def get_active_matches_count():
    return {"count": len(active_matches)}

@app.post("/api/results")
async def add_result(res: GameResult):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        INSERT INTO game_results (red_team_id, blue_team_id, red_team_name, blue_team_name, score_red, score_blue, game_time)
        VALUES (?, ?, ?, ?, ?, ?, ?)
    """, (res.red_team_id, res.blue_team_id, res.red_team_name, res.blue_team_name, res.score_red, res.score_blue, res.game_time))
    conn.commit()
    conn.close()
    return {"status": "ok"}

@app.get("/api/matches/active")
async def get_active_matches():
    return active_matches

@app.post("/api/matches/clear-all")
async def clear_all_pairings():
    for pairing in active_matches.values():
        proxy.clear_route(pairing.red_team_id)
        proxy.clear_route(pairing.blue_team_id)
    active_matches.clear()
    return {"status": "ok"}

# --- API Endpoints: Results ---

@app.get("/api/results")
async def get_results():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM game_results ORDER BY created_at DESC")
    rows = cursor.fetchall()
    results = [dict(r) for r in rows]
    conn.close()
    return results

@app.post("/api/matches/auto-assign")
async def auto_assign():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT red_team_id, blue_team_id FROM game_results")
    played = set(cursor.fetchall())
    conn.close()

    available_teams = [id for id, t in teams.items() if t.enabled]
    available_sims = [id for id, s in simulators.items() if s.enabled and id not in active_matches]
    
    if not available_sims:
        return {"status": "no_available_sims"}

    to_play = []
    for t1 in available_teams:
        for t2 in available_teams:
            if t1 == t2: continue
            if (t1, t2) not in played:
                to_play.append((t1, t2))
    
    if not to_play:
        return {"status": "all_played"}

    assigned_count = 0
    currently_playing = set()
    for m in active_matches.values():
        currently_playing.add(m.red_team_id)
        currently_playing.add(m.blue_team_id)

    for sim_id in available_sims:
        for t1, t2 in to_play:
            if t1 not in currently_playing and t2 not in currently_playing:
                await assign_match(MatchPairing(sim_id=sim_id, red_team_id=t1, blue_team_id=t2))
                currently_playing.add(t1)
                currently_playing.add(t2)
                assigned_count += 1
                break

    return {"status": "ok", "assigned": assigned_count}

@app.put("/api/results/{result_id}")
async def update_result(result_id: int, score_red: int = Query(...), score_blue: int = Query(...)):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("UPDATE game_results SET score_red = ?, score_blue = ? WHERE id = ?", (score_red, score_blue, result_id))
    conn.commit()
    conn.close()
    return {"status": "updated"}

@app.delete("/api/results/{result_id}")
async def delete_result(result_id: int):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("DELETE FROM game_results WHERE id = ?", (result_id,))
    conn.commit()
    conn.close()
    return {"status": "deleted"}

@app.post("/api/results/clear-all")
async def clear_all_results():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("DELETE FROM game_results")
    conn.commit()
    conn.close()
    return {"status": "cleared"}


# ============================================================
# QUALIFICATION STANDINGS — Duel-based scoring (rules §2.2)
# ============================================================
# Each pair plays 2 games (A as red vs B, then B as red vs A).
# Duel winner = team with more total goals across both games.
# Win=3pts, Draw=1pt, Loss=0pts.
# Ranking: (1) points, (2) goal difference, (3) goals scored.
# ============================================================

def compute_qualification_standings():
    """Compute duel-based qualification standings from game_results."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM game_results ORDER BY created_at ASC")
    results = [dict(r) for r in cursor.fetchall()]
    conn.close()

    # Build per-team stats
    team_stats = {}  # team_id -> {name, pts, gf, ga, duels_played}

    # Group games by unordered pair
    from collections import defaultdict
    duels = defaultdict(list)  # (min_id, max_id) -> list of game dicts

    for r in results:
        rid, bid = r["red_team_id"], r["blue_team_id"]
        key = (min(rid, bid), max(rid, bid))
        duels[key].append(r)

        # Initialize team stats
        for tid, tname in [(rid, r["red_team_name"]), (bid, r["blue_team_name"])]:
            if tid not in team_stats:
                team_stats[tid] = {"id": tid, "name": tname, "pts": 0, "goals_for": 0, "goals_against": 0, "duels_played": 0, "wins": 0, "draws": 0, "losses": 0}

    # Process each duel
    processed_duels = set()
    for (t_a, t_b), games in duels.items():
        if len(games) < 2:
            # Duel not complete yet — don't count toward standings
            # But still track goals for informational purposes
            for g in games:
                rid, bid = g["red_team_id"], g["blue_team_id"]
                if rid in team_stats:
                    team_stats[rid]["goals_for"] += g["score_red"]
                    team_stats[rid]["goals_against"] += g["score_blue"]
                if bid in team_stats:
                    team_stats[bid]["goals_for"] += g["score_blue"]
                    team_stats[bid]["goals_against"] += g["score_red"]
            continue

        # Take first 2 games of this duel
        g1, g2 = games[0], games[1]

        # Calculate total goals for each team across both games
        goals = {t_a: 0, t_b: 0}
        for g in [g1, g2]:
            rid, bid = g["red_team_id"], g["blue_team_id"]
            goals[rid] = goals.get(rid, 0) + g["score_red"]
            goals[bid] = goals.get(bid, 0) + g["score_blue"]

        # Track goals
        for g in [g1, g2]:
            rid, bid = g["red_team_id"], g["blue_team_id"]
            team_stats[rid]["goals_for"] += g["score_red"]
            team_stats[rid]["goals_against"] += g["score_blue"]
            team_stats[bid]["goals_for"] += g["score_blue"]
            team_stats[bid]["goals_against"] += g["score_red"]

        # Determine duel winner
        team_stats[t_a]["duels_played"] += 1
        team_stats[t_b]["duels_played"] += 1

        if goals[t_a] > goals[t_b]:
            team_stats[t_a]["pts"] += 3
            team_stats[t_a]["wins"] += 1
            team_stats[t_b]["losses"] += 1
        elif goals[t_b] > goals[t_a]:
            team_stats[t_b]["pts"] += 3
            team_stats[t_b]["wins"] += 1
            team_stats[t_a]["losses"] += 1
        else:
            team_stats[t_a]["pts"] += 1
            team_stats[t_a]["draws"] += 1
            team_stats[t_b]["pts"] += 1
            team_stats[t_b]["draws"] += 1

    # Sort by: (1) points desc, (2) goal difference desc, (3) goals scored desc
    sorted_standings = sorted(
        team_stats.values(),
        key=lambda x: (x["pts"], x["goals_for"] - x["goals_against"], x["goals_for"]),
        reverse=True
    )

    # Add rank and goal_diff
    for i, s in enumerate(sorted_standings):
        s["rank"] = i + 1
        s["goal_diff"] = s["goals_for"] - s["goals_against"]

    return sorted_standings


@app.get("/api/results/standings")
async def get_standings():
    return compute_qualification_standings()


# ============================================================
# TOURNAMENT API ENDPOINTS
# ============================================================

ROUND_NAMES = ["round_of_16", "quarter_final", "semi_final", "third_place", "final"]
ROUND_DISPLAY = {
    "round_of_16": "Osmina finala",
    "quarter_final": "Četrtfinale",
    "semi_final": "Polfinale",
    "third_place": "Tekma za 3. mesto",
    "final": "Finale",
}
ROUND_MATCH_COUNTS = {
    "round_of_16": 8,
    "quarter_final": 4,
    "semi_final": 2,
    "third_place": 1,
    "final": 1,
}


def get_tournament_phase():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT phase, confirmed FROM tournament_config WHERE id = 1")
    row = cursor.fetchone()
    conn.close()
    if row:
        return {"phase": row[0], "confirmed": bool(row[1])}
    return {"phase": "preparation", "confirmed": False}


def set_tournament_phase(phase: str, confirmed: bool = False):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute(
        "UPDATE tournament_config SET phase = ?, confirmed = ?, updated_at = CURRENT_TIMESTAMP WHERE id = 1",
        (phase, int(confirmed))
    )
    conn.commit()
    conn.close()


def get_head_to_head_goals(team1_id: int, team2_id: int):
    """Get head-to-head qualification goals between two teams."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    cursor.execute("""
        SELECT red_team_id, blue_team_id, score_red, score_blue
        FROM game_results
        WHERE (red_team_id = ? AND blue_team_id = ?) OR (red_team_id = ? AND blue_team_id = ?)
    """, (team1_id, team2_id, team2_id, team1_id))
    games = cursor.fetchall()
    conn.close()

    t1_goals = 0
    t2_goals = 0
    for g in games:
        if g["red_team_id"] == team1_id:
            t1_goals += g["score_red"]
            t2_goals += g["score_blue"]
        else:
            t2_goals += g["score_red"]
            t1_goals += g["score_blue"]

    return t1_goals, t2_goals


@app.get("/api/tournament/state")
async def tournament_state():
    state = get_tournament_phase()
    return state


@app.post("/api/tournament/generate-pairs")
async def generate_tournament_pairs():
    """Generate round of 16 pairs from qualification standings (1v16, 2v15, etc.)"""
    state = get_tournament_phase()
    if state["confirmed"]:
        raise HTTPException(status_code=400, detail="Tournament already confirmed")

    standings = compute_qualification_standings()
    if len(standings) < 16:
        raise HTTPException(status_code=400, detail=f"Need at least 16 teams, have {len(standings)}")

    top16 = standings[:16]

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Clear existing preparation pairs
    cursor.execute("DELETE FROM tournament_games WHERE pair_id IN (SELECT id FROM tournament_pairs)")
    cursor.execute("DELETE FROM tournament_pairs")

    # Generate round of 16 pairs: 1v16, 2v15, 3v14, ..., 8v9
    for i in range(8):
        t1 = top16[i]
        t2 = top16[15 - i]

        cursor.execute("""
            INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """, ("round_of_16", i, t1["id"], t2["id"], t1["name"], t2["name"], t1["rank"], t2["rank"]))

        pair_id = cursor.lastrowid
        # Create 2 games per pair
        cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 1, 'pending')", (pair_id,))
        cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 2, 'pending')", (pair_id,))

    set_tournament_phase("preparation", False)
    conn.commit()
    conn.close()

    return {"status": "generated", "pairs": 8}


@app.get("/api/tournament/preparation")
async def get_tournament_preparation():
    """Get current tournament pairs for editing."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM tournament_pairs WHERE round = 'round_of_16' ORDER BY match_index")
    pairs = [dict(r) for r in cursor.fetchall()]
    conn.close()

    standings = compute_qualification_standings()

    return {
        "pairs": pairs,
        "standings": standings,
        "phase": get_tournament_phase(),
    }


@app.post("/api/tournament/preparation/update-pair")
async def update_tournament_pair(update: TournamentPairUpdate):
    """Update a pair in preparation phase."""
    state = get_tournament_phase()
    if state["confirmed"]:
        raise HTTPException(status_code=400, detail="Tournament already confirmed")

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    if update.team1_id is not None:
        # Look up team name
        name = teams[update.team1_id].name if update.team1_id in teams else f"Team {update.team1_id}"
        cursor.execute("UPDATE tournament_pairs SET team1_id = ?, team1_name = ? WHERE id = ?",
                       (update.team1_id, name, update.pair_id))
    if update.team2_id is not None:
        name = teams[update.team2_id].name if update.team2_id in teams else f"Team {update.team2_id}"
        cursor.execute("UPDATE tournament_pairs SET team2_id = ?, team2_name = ? WHERE id = ?",
                       (update.team2_id, name, update.pair_id))

    conn.commit()
    conn.close()
    return {"status": "updated"}


@app.post("/api/tournament/preparation/add-pair")
async def add_tournament_pair(team1_id: int, team2_id: int):
    """Add a new pair to the round of 16."""
    state = get_tournament_phase()
    if state["confirmed"]:
        raise HTTPException(status_code=400, detail="Tournament already confirmed")

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Get next match index
    cursor.execute("SELECT COALESCE(MAX(match_index), -1) + 1 FROM tournament_pairs WHERE round = 'round_of_16'")
    next_idx = cursor.fetchone()[0]

    t1_name = teams[team1_id].name if team1_id in teams else f"Team {team1_id}"
    t2_name = teams[team2_id].name if team2_id in teams else f"Team {team2_id}"

    cursor.execute("""
        INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed)
        VALUES ('round_of_16', ?, ?, ?, ?, ?, 0, 0)
    """, (next_idx, team1_id, team2_id, t1_name, t2_name))

    pair_id = cursor.lastrowid
    cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 1, 'pending')", (pair_id,))
    cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 2, 'pending')", (pair_id,))

    conn.commit()
    conn.close()
    return {"status": "added", "pair_id": pair_id}


@app.delete("/api/tournament/preparation/remove-pair/{pair_id}")
async def remove_tournament_pair(pair_id: int):
    """Remove a pair from preparation."""
    state = get_tournament_phase()
    if state["confirmed"]:
        raise HTTPException(status_code=400, detail="Tournament already confirmed")

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("DELETE FROM tournament_games WHERE pair_id = ?", (pair_id,))
    cursor.execute("DELETE FROM tournament_pairs WHERE id = ?", (pair_id,))
    conn.commit()
    conn.close()
    return {"status": "removed"}


@app.post("/api/tournament/confirm")
async def confirm_tournament():
    """Lock in the tournament pairs and start the tournament."""
    state = get_tournament_phase()
    if state["confirmed"]:
        raise HTTPException(status_code=400, detail="Tournament already confirmed")

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT COUNT(*) FROM tournament_pairs WHERE round = 'round_of_16'")
    count = cursor.fetchone()[0]
    conn.close()

    if count == 0:
        raise HTTPException(status_code=400, detail="No pairs to confirm")

    set_tournament_phase("active", True)
    return {"status": "confirmed", "pairs": count}


@app.get("/api/tournament/bracket")
async def get_tournament_bracket():
    """Get the full tournament bracket with all rounds and results."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    bracket = {}
    for round_name in ROUND_NAMES:
        cursor.execute("""
            SELECT tp.*, 
                   tp.id as pair_id
            FROM tournament_pairs tp
            WHERE tp.round = ?
            ORDER BY tp.match_index
        """, (round_name,))
        pairs = []
        for p in cursor.fetchall():
            p_dict = dict(p)

            # Get games for this pair
            cursor.execute("""
                SELECT * FROM tournament_games WHERE pair_id = ? ORDER BY game_number
            """, (p_dict["id"],))
            p_dict["games"] = [dict(g) for g in cursor.fetchall()]

            # Calculate duel score with qualification advantage
            p_dict["duel_score"] = compute_duel_score(p_dict, cursor)

            pairs.append(p_dict)

        bracket[round_name] = {
            "display_name": ROUND_DISPLAY.get(round_name, round_name),
            "pairs": pairs,
        }

    conn.close()

    return {
        "state": get_tournament_phase(),
        "bracket": bracket,
    }


def compute_duel_score(pair_dict, cursor=None):
    """Compute the tournament duel score including qualification advantage.

    Rules §2.3:
    - Each tournament goal = 1 point
    - Each qualification goal (from head-to-head between same teams) = 0.5 points
    - Tiebreaker: (1) total points, (2) qual duel winner, (3) better qual rank
    """
    t1_id = pair_dict.get("team1_id")
    t2_id = pair_dict.get("team2_id")
    if not t1_id or not t2_id:
        return None

    games = pair_dict.get("games", [])
    completed_games = [g for g in games if g["status"] == "completed"]

    # Tournament goals (1pt each)
    t1_tournament_goals = sum(g["team1_score"] for g in completed_games)
    t2_tournament_goals = sum(g["team2_score"] for g in completed_games)

    # Qualification head-to-head goals (0.5pt each)
    t1_qual_goals, t2_qual_goals = get_head_to_head_goals(t1_id, t2_id)

    t1_total = t1_tournament_goals * 1.0 + t1_qual_goals * 0.5
    t2_total = t2_tournament_goals * 1.0 + t2_qual_goals * 0.5

    # Determine qual duel winner for tiebreaker
    qual_duel_winner = None
    if t1_qual_goals > t2_qual_goals:
        qual_duel_winner = t1_id
    elif t2_qual_goals > t1_qual_goals:
        qual_duel_winner = t2_id

    return {
        "team1_tournament_goals": t1_tournament_goals,
        "team2_tournament_goals": t2_tournament_goals,
        "team1_qual_goals": t1_qual_goals,
        "team2_qual_goals": t2_qual_goals,
        "team1_total_points": t1_total,
        "team2_total_points": t2_total,
        "qual_duel_winner": qual_duel_winner,
        "team1_seed": pair_dict.get("team1_seed"),
        "team2_seed": pair_dict.get("team2_seed"),
    }


@app.get("/api/tournament/games")
async def get_tournament_games():
    """Get all tournament games grouped by status."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    cursor.execute("""
        SELECT tg.*, tp.round, tp.match_index, tp.team1_id, tp.team2_id, 
               tp.team1_name, tp.team2_name, tp.team1_seed, tp.team2_seed,
               tp.id as pair_id
        FROM tournament_games tg
        JOIN tournament_pairs tp ON tg.pair_id = tp.id
        ORDER BY 
            CASE tg.status 
                WHEN 'active' THEN 0 
                WHEN 'pending' THEN 1 
                WHEN 'completed' THEN 2 
            END,
            CASE tp.round
                WHEN 'round_of_16' THEN 0
                WHEN 'quarter_final' THEN 1
                WHEN 'semi_final' THEN 2
                WHEN 'third_place' THEN 3
                WHEN 'final' THEN 4
            END,
            tp.match_index, tg.game_number
    """)
    games = [dict(g) for g in cursor.fetchall()]
    conn.close()

    return {
        "active": [g for g in games if g["status"] == "active"],
        "pending": [g for g in games if g["status"] == "pending"],
        "completed": [g for g in games if g["status"] == "completed"],
    }


@app.post("/api/tournament/game/{game_id}/set-active")
async def set_game_active(game_id: int):
    """Set a game as the active game."""
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Deactivate any currently active game
    cursor.execute("UPDATE tournament_games SET status = 'pending' WHERE status = 'active'")

    # Activate the selected game
    cursor.execute("UPDATE tournament_games SET status = 'active' WHERE id = ? AND status != 'completed'", (game_id,))
    if cursor.rowcount == 0:
        conn.close()
        raise HTTPException(status_code=400, detail="Game not found or already completed")

    conn.commit()
    conn.close()
    return {"status": "activated"}


@app.post("/api/tournament/game/{game_id}/result")
async def submit_game_result(game_id: int, result: TournamentGameResult):
    """Submit the result for a tournament game."""
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    cursor.execute("SELECT status FROM tournament_games WHERE id = ?", (game_id,))
    row = cursor.fetchone()
    if not row:
        conn.close()
        raise HTTPException(status_code=404, detail="Game not found")
    if row[0] == "completed":
        conn.close()
        raise HTTPException(status_code=400, detail="Game already completed")

    cursor.execute("""
        UPDATE tournament_games 
        SET team1_score = ?, team2_score = ?, game_time = ?, status = 'active'
        WHERE id = ?
    """, (result.team1_score, result.team2_score, result.game_time, game_id))

    conn.commit()
    conn.close()
    return {"status": "result_saved"}


@app.post("/api/tournament/game/{game_id}/confirm")
async def confirm_game_result(game_id: int):
    """Confirm the game result and check if we need to advance the bracket."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    # Get game info
    cursor.execute("""
        SELECT tg.*, tp.round, tp.match_index, tp.team1_id, tp.team2_id,
               tp.team1_name, tp.team2_name, tp.team1_seed, tp.team2_seed,
               tp.id as pair_id_val
        FROM tournament_games tg
        JOIN tournament_pairs tp ON tg.pair_id = tp.id
        WHERE tg.id = ?
    """, (game_id,))
    game = cursor.fetchone()
    if not game:
        conn.close()
        raise HTTPException(status_code=404, detail="Game not found")

    game = dict(game)

    # Mark game as completed
    cursor.execute("UPDATE tournament_games SET status = 'completed' WHERE id = ?", (game_id,))

    # Check if all games in this pair are completed
    cursor.execute("""
        SELECT COUNT(*) FROM tournament_games 
        WHERE pair_id = ? AND status != 'completed'
    """, (game["pair_id"],))
    remaining = cursor.fetchone()[0]

    if remaining == 0:
        # All games in this pair are done — determine winner
        pair_id = game["pair_id"]
        cursor.execute("SELECT * FROM tournament_pairs WHERE id = ?", (pair_id,))
        pair = dict(cursor.fetchone())

        cursor.execute("SELECT * FROM tournament_games WHERE pair_id = ? ORDER BY game_number", (pair_id,))
        pair["games"] = [dict(g) for g in cursor.fetchall()]

        duel = compute_duel_score(pair)
        winner_id = determine_winner(pair, duel)

        cursor.execute("UPDATE tournament_pairs SET winner_id = ? WHERE id = ?", (winner_id, pair_id))

        # Check if all pairs in this round are done, then generate next round
        current_round = game["round"]
        cursor.execute("""
            SELECT COUNT(*) FROM tournament_pairs 
            WHERE round = ? AND winner_id IS NULL
        """, (current_round,))
        round_remaining = cursor.fetchone()[0]

        if round_remaining == 0:
            advance_bracket(cursor, current_round)

    conn.commit()
    conn.close()

    # Notify visualization
    await broadcast_viz_update()

    return {"status": "confirmed"}


def determine_winner(pair_dict, duel_score):
    """Determine the winner of a tournament pair per rules §2.3."""
    if not duel_score:
        return None

    t1_id = pair_dict["team1_id"]
    t2_id = pair_dict["team2_id"]

    # (1) Total points
    if duel_score["team1_total_points"] > duel_score["team2_total_points"]:
        return t1_id
    elif duel_score["team2_total_points"] > duel_score["team1_total_points"]:
        return t2_id

    # (2) Qualification duel winner
    if duel_score["qual_duel_winner"] == t1_id:
        return t1_id
    elif duel_score["qual_duel_winner"] == t2_id:
        return t2_id

    # (3) Better qualification rank (lower seed = better)
    t1_seed = duel_score.get("team1_seed") or 999
    t2_seed = duel_score.get("team2_seed") or 999
    if t1_seed < t2_seed:
        return t1_id
    elif t2_seed < t1_seed:
        return t2_id

    # Fallback: team1 wins
    return t1_id


def advance_bracket(cursor, completed_round: str):
    """Generate the next round's pairs from the completed round's winners."""
    round_progression = {
        "round_of_16": "quarter_final",
        "quarter_final": "semi_final",
        "semi_final": None,  # special: generates both third_place and final
    }

    if completed_round == "semi_final":
        # Get semi-final winners and losers
        cursor.execute("""
            SELECT * FROM tournament_pairs WHERE round = 'semi_final' ORDER BY match_index
        """)
        semis = [dict(r) for r in cursor.fetchall()]

        winners = []
        losers = []
        for s in semis:
            if s["winner_id"] == s["team1_id"]:
                winners.append({"id": s["team1_id"], "name": s["team1_name"], "seed": s["team1_seed"]})
                losers.append({"id": s["team2_id"], "name": s["team2_name"], "seed": s["team2_seed"]})
            else:
                winners.append({"id": s["team2_id"], "name": s["team2_name"], "seed": s["team2_seed"]})
                losers.append({"id": s["team1_id"], "name": s["team1_name"], "seed": s["team1_seed"]})

        # Create third place match
        if len(losers) >= 2:
            cursor.execute("""
                INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed)
                VALUES ('third_place', 0, ?, ?, ?, ?, ?, ?)
            """, (losers[0]["id"], losers[1]["id"], losers[0]["name"], losers[1]["name"], losers[0]["seed"], losers[1]["seed"]))
            pid = cursor.lastrowid
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 1, 'pending')", (pid,))
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 2, 'pending')", (pid,))

        # Create final
        if len(winners) >= 2:
            cursor.execute("""
                INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed)
                VALUES ('final', 0, ?, ?, ?, ?, ?, ?)
            """, (winners[0]["id"], winners[1]["id"], winners[0]["name"], winners[1]["name"], winners[0]["seed"], winners[1]["seed"]))
            pid = cursor.lastrowid
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 1, 'pending')", (pid,))
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 2, 'pending')", (pid,))

        return

    next_round = round_progression.get(completed_round)
    if not next_round:
        return

    # Get winners from completed round in order
    cursor.execute("""
        SELECT * FROM tournament_pairs WHERE round = ? ORDER BY match_index
    """, (completed_round,))
    pairs = [dict(r) for r in cursor.fetchall()]

    # Winners pair up: 0+1, 2+3, etc.
    for i in range(0, len(pairs), 2):
        if i + 1 >= len(pairs):
            break

        p1 = pairs[i]
        p2 = pairs[i + 1]

        # Get winner info
        def get_winner_info(p):
            if p["winner_id"] == p["team1_id"]:
                return {"id": p["team1_id"], "name": p["team1_name"], "seed": p["team1_seed"]}
            return {"id": p["team2_id"], "name": p["team2_name"], "seed": p["team2_seed"]}

        w1 = get_winner_info(p1)
        w2 = get_winner_info(p2)

        cursor.execute("""
            INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """, (next_round, i // 2, w1["id"], w2["id"], w1["name"], w2["name"], w1["seed"], w2["seed"]))

        pid = cursor.lastrowid
        cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 1, 'pending')", (pid,))
        cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 2, 'pending')", (pid,))


@app.post("/api/tournament/reset")
async def reset_tournament():
    """Reset the entire tournament (preparation phase)."""
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("DELETE FROM tournament_games")
    cursor.execute("DELETE FROM tournament_pairs")
    cursor.execute("UPDATE tournament_config SET phase = 'preparation', confirmed = 0 WHERE id = 1")
    conn.commit()
    conn.close()
    return {"status": "reset"}


# ============================================================
# VISUALIZATION SSE & CONTROL API
# ============================================================

@app.get("/api/tournament/viz-state")
async def get_viz_state():
    """Get current visualization state."""
    return viz_state


@app.post("/api/tournament/viz-state")
async def update_viz_state(update: VizStateUpdate):
    """Update visualization state (from viz-control page)."""
    global viz_state

    if update.view is not None:
        viz_state["view"] = update.view
    if update.match_id is not None:
        viz_state["match_id"] = update.match_id
    if update.round_name is not None:
        viz_state["round_name"] = update.round_name
    if update.team_id is not None:
        viz_state["team_id"] = update.team_id
    if update.live is not None:
        viz_state["live"] = update.live
    if update.message is not None:
        viz_state["message"] = update.message

    viz_state["tiktak"] += 1

    await broadcast_viz_update()
    return viz_state


async def broadcast_viz_update():
    """Broadcast current viz state to all SSE subscribers."""
    msg = json.dumps({"type": "viz_update", "data": viz_state})
    dead = []
    for q in viz_subscribers:
        try:
            q.put_nowait(msg)
        except asyncio.QueueFull:
            dead.append(q)
    for q in dead:
        viz_subscribers.remove(q)


@app.get("/api/tournament/viz-stream")
async def viz_sse_stream(request: Request):
    """SSE stream for visualization clients."""
    from starlette.responses import StreamingResponse

    q = asyncio.Queue(maxsize=50)
    viz_subscribers.append(q)

    async def event_generator():
        try:
            # Send initial state
            yield f"data: {json.dumps({'type': 'viz_update', 'data': viz_state})}\n\n"

            while True:
                # Check if client disconnected
                if await request.is_disconnected():
                    break
                try:
                    msg = await asyncio.wait_for(q.get(), timeout=15.0)
                    yield f"data: {msg}\n\n"
                except asyncio.TimeoutError:
                    # Send keepalive
                    yield f": keepalive\n\n"
        except asyncio.CancelledError:
            pass
        finally:
            if q in viz_subscribers:
                viz_subscribers.remove(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


# --- Static Files & Routing ---

if not os.path.exists(WWW_DIR):
    os.makedirs(WWW_DIR)

app.mount("/static", StaticFiles(directory=WWW_DIR), name="static")

# Landing page = visualization
@app.get("/")
async def serve_root():
    return FileResponse(os.path.join(WWW_DIR, "visualization.html"))

@app.get("/visualization")
@app.get("/visualization.html")
async def serve_visualization():
    return FileResponse(os.path.join(WWW_DIR, "visualization.html"))

@app.get("/qualifications")
@app.get("/qualifications.html")
async def serve_qualifications():
    return FileResponse(os.path.join(WWW_DIR, "index.html"))

@app.get("/tournament")
@app.get("/tournament.html")
async def serve_tournament():
    return FileResponse(os.path.join(WWW_DIR, "tournament.html"))

@app.get("/viz-control")
@app.get("/viz-control.html")
async def serve_viz_control():
    return FileResponse(os.path.join(WWW_DIR, "viz-control.html"))

@app.get("/{path:path}")
async def serve_spa(path: str):
    local_path = os.path.join(WWW_DIR, path)
    if os.path.exists(local_path) and os.path.isfile(local_path):
        return FileResponse(local_path)
    # Fallback to visualization
    return FileResponse(os.path.join(WWW_DIR, "visualization.html"))

# --- Config Management Endpoints ---

@app.post("/api/config/save")
async def api_save_config():
    save_config()
    return {"status": "saved"}

@app.post("/api/config/load")
async def api_load_config():
    load_config()
    return {"status": "loaded"}

# --- Main ---

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=30400)
    args = parser.parse_args()
    
    # Load config to get allowed_ips before adding middleware
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r") as f:
            data = json.load(f)
            allowed_ips = data.get("allowed_ips", [])

    app.add_middleware(IPWhitelistMiddleware, allowed_ips=allowed_ips)

    logger.info("Starting FuzbAI Competition Manager v2 on port %d", args.port)
    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")
