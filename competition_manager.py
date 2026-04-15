import asyncio
import json
import logging
import math
import os
import shutil
import sqlite3
import time
import uuid
from collections import defaultdict
from contextlib import asynccontextmanager
from typing import List, Optional

import httpx
import uvicorn
from fastapi import FastAPI, HTTPException, Request, Query, UploadFile, File
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
PHOTOS_DIR = os.path.join(WWW_DIR, "photos")
os.makedirs(PHOTOS_DIR, exist_ok=True)

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
        "/api/tournament/viz-stream",
        "/api/tournament/games",
        "/api/teams/details",
        "/api/results",
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
    picture_url: str = ""
    algorithm_description: str = ""
    members: List[str] = []

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
    view: Optional[str] = None          # "bracket", "match", "standings", "team", "title", "teams", "matrix"
    match_id: Optional[int] = None      # which match to show
    round_name: Optional[str] = None    # which round to focus
    team_id: Optional[int] = None       # which team to show
    live: Optional[bool] = None         # live indicator
    message: Optional[str] = None       # overlay message

class GoalAction(BaseModel):
    team: int  # 1 or 2

class FoulAction(BaseModel):
    team: int  # 1 or 2

class OverrideScore(BaseModel):
    team1_total: float
    team2_total: float

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
            team_count INTEGER NOT NULL DEFAULT 8,
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
            override_team1_total REAL DEFAULT NULL,
            override_team2_total REAL DEFAULT NULL,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)
    try:
        cursor.execute("ALTER TABLE tournament_pairs ADD COLUMN override_team1_total REAL DEFAULT NULL")
    except:
        pass
    try:
        cursor.execute("ALTER TABLE tournament_pairs ADD COLUMN override_team2_total REAL DEFAULT NULL")
    except:
        pass

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS tournament_games (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            pair_id INTEGER NOT NULL,
            game_number INTEGER NOT NULL,
            team1_score INTEGER DEFAULT 0,
            team2_score INTEGER DEFAULT 0,
            team1_fouls INTEGER DEFAULT 0,
            team2_fouls INTEGER DEFAULT 0,
            game_time REAL DEFAULT 0,
            status TEXT NOT NULL DEFAULT 'pending',
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            FOREIGN KEY (pair_id) REFERENCES tournament_pairs(id)
        )
    """)
    try:
        cursor.execute("ALTER TABLE tournament_games ADD COLUMN team1_fouls INTEGER DEFAULT 0")
    except:
        pass
    try:
        cursor.execute("ALTER TABLE tournament_games ADD COLUMN team2_fouls INTEGER DEFAULT 0")
    except:
        pass

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

@app.get("/api/teams/details")
async def get_teams_details():
    """Public endpoint: returns team id, name, picture_url, algorithm_description, members."""
    return [
        {"id": tid, "name": t.name, "picture_url": t.picture_url, "algorithm_description": t.algorithm_description, "members": t.members}
        for tid, t in teams.items()
    ]

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

class TeamDetailsUpdate(BaseModel):
    name: Optional[str] = None
    picture_url: Optional[str] = None
    algorithm_description: Optional[str] = None
    members: Optional[List[str]] = None

@app.put("/api/teams/{team_id}/details")
async def update_team_details(team_id: int, details: TeamDetailsUpdate):
    if team_id not in teams:
        raise HTTPException(status_code=404, detail="Team not found")
    if details.name is not None:
        teams[team_id].name = details.name
    if details.picture_url is not None:
        teams[team_id].picture_url = details.picture_url
    if details.algorithm_description is not None:
        teams[team_id].algorithm_description = details.algorithm_description
    if details.members is not None:
        teams[team_id].members = details.members[:3]  # max 3 members
    save_config()
    return {"status": "ok"}

ALLOWED_IMAGE_TYPES = {"image/jpeg", "image/png", "image/gif", "image/webp"}
MAX_IMAGE_SIZE = 5 * 1024 * 1024  # 5 MB

@app.post("/api/teams/{team_id}/photo")
async def upload_team_photo(team_id: int, file: UploadFile = File(...)):
    if team_id not in teams:
        raise HTTPException(status_code=404, detail="Team not found")
    if file.content_type not in ALLOWED_IMAGE_TYPES:
        raise HTTPException(status_code=400, detail=f"Nepodprt tip datoteke: {file.content_type}. Dovoljeni: JPEG, PNG, GIF, WebP.")
    contents = await file.read()
    if len(contents) > MAX_IMAGE_SIZE:
        raise HTTPException(status_code=400, detail="Datoteka je prevelika (maks. 5 MB).")
    ext = file.filename.rsplit('.', 1)[-1].lower() if '.' in file.filename else 'jpg'
    if ext not in ('jpg', 'jpeg', 'png', 'gif', 'webp'):
        ext = 'jpg'
    filename = f"team_{team_id}_{uuid.uuid4().hex[:8]}.{ext}"
    filepath = os.path.join(PHOTOS_DIR, filename)
    # Remove old uploaded photo if it was in photos dir
    old_url = teams[team_id].picture_url or ''
    if old_url.startswith('/static/photos/') or old_url.startswith('/photos/'):
        old_file = os.path.join(WWW_DIR, old_url.lstrip('/').replace('static/', '', 1) if old_url.startswith('/static/') else old_url.lstrip('/'))
        if os.path.exists(old_file):
            os.remove(old_file)
    with open(filepath, 'wb') as f:
        f.write(contents)
    photo_url = f"/static/photos/{filename}"
    teams[team_id].picture_url = photo_url
    save_config()
    return {"status": "ok", "picture_url": photo_url}

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

    # Notify visualization clients about new results
    await broadcast_viz_update()

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

    # Notify visualization clients about new results
    await broadcast_viz_update()

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
    await broadcast_viz_update()
    return {"status": "updated"}

@app.delete("/api/results/{result_id}")
async def delete_result(result_id: int):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("DELETE FROM game_results WHERE id = ?", (result_id,))
    conn.commit()
    conn.close()
    await broadcast_viz_update()
    return {"status": "deleted"}

@app.post("/api/results/clear-all")
async def clear_all_results():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("DELETE FROM game_results")
    conn.commit()
    conn.close()
    await broadcast_viz_update()
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

# --- Dynamic round name helpers ---
# Instead of hardcoded round names for 16 teams, we generate them
# dynamically based on how many teams enter the tournament.

def next_power_of_2(n: int) -> int:
    """Return the smallest power of 2 >= n."""
    if n <= 1:
        return 1
    return 1 << (n - 1).bit_length()


def build_round_info(num_teams: int):
    """
    Build round names, display names, and match counts for a single-elimination
    bracket with `num_teams` teams.  Padded to next power of 2 (byes fill gaps).

    Returns (round_names, round_display, round_match_counts).
    round_names is ordered from first round to semi_final (excluding third_place/final,
    which are always the last two and handled specially).
    """
    bracket_size = next_power_of_2(num_teams)  # e.g. 5->8, 8->8, 10->16
    num_rounds = int(math.log2(bracket_size))  # e.g. 8->3, 16->4

    # Map: number of matches in round -> canonical round name & display
    CANONICAL = {
        1: ("final", "Finale"),         # not used in main list – always appended
        2: ("semi_final", "Polfinale"),
        4: ("quarter_final", "Četrtfinale"),
        8: ("round_of_16", "Osmina finala"),
        16: ("round_of_32", "Šestnajstina finala"),
        32: ("round_of_64", "Dvaintrisetina finala"),
    }

    round_names = []     # ordered from first round to semi_final
    round_display = {}   # round_name -> Slovenian display
    round_match_counts = {}  # round_name -> number of matches

    matches = bracket_size // 2   # first round matches
    for r in range(num_rounds):
        if matches < 1:
            break
        # Last round (matches==1) is final – handled separately
        if matches == 1:
            break
        if matches in CANONICAL:
            rname, rdisplay = CANONICAL[matches]
        else:
            rname = f"round_of_{matches * 2}"
            rdisplay = f"Krog {matches * 2}"
        round_names.append(rname)
        round_display[rname] = rdisplay
        round_match_counts[rname] = matches
        matches //= 2

    # Always add third_place and final at the end
    round_names.append("third_place")
    round_display["third_place"] = "Tekma za 3. mesto"
    round_match_counts["third_place"] = 1

    round_names.append("final")
    round_display["final"] = "Finale"
    round_match_counts["final"] = 1

    return round_names, round_display, round_match_counts


def get_first_round_name(num_teams: int) -> str:
    """Return the name of the first round for this bracket size."""
    rn, _, _ = build_round_info(num_teams)
    return rn[0]


def get_bracket_round_names():
    """Get round names from the stored team_count in tournament_config."""
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT team_count FROM tournament_config WHERE id = 1")
    row = cursor.fetchone()
    conn.close()
    team_count = row[0] if row else 8
    return build_round_info(team_count)


def get_tournament_phase():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT phase, confirmed FROM tournament_config WHERE id = 1")
    row = cursor.fetchone()
    conn.close()
    if row:
        return {"phase": row[0], "confirmed": bool(row[1])}
    return {"phase": "preparation", "confirmed": False}


def set_tournament_phase(phase: str, confirmed: bool = False, cursor=None):
    """Update tournament phase. Optionally accepts an existing cursor to avoid DB lock."""
    own_conn = cursor is None
    if own_conn:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
    cursor.execute(
        "UPDATE tournament_config SET phase = ?, confirmed = ?, updated_at = CURRENT_TIMESTAMP WHERE id = 1",
        (phase, int(confirmed))
    )
    if own_conn:
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
async def generate_tournament_pairs(team_count: int = Query(default=8, ge=2)):
    """Generate first-round pairs from qualification standings for any number of teams.
    
    Builds a proper single-elimination bracket:
    - Pads to next power of 2 (e.g. 5 teams → 8 bracket slots)
    - Top seeds get byes (auto-advance to next round)
    - Seeding: 1 vs N, 2 vs N-1, etc. within the bracket structure
    
    Primarily designed for 8 teams, but works with any count.
    """
    state = get_tournament_phase()
    if state["confirmed"]:
        raise HTTPException(status_code=400, detail="Tournament already confirmed")

    standings = compute_qualification_standings()
    if len(standings) == 0:
        raise HTTPException(
            status_code=400,
            detail="No teams found in qualification standings"
        )

    # If fewer teams than requested, use all available teams
    actual_count = min(team_count, len(standings))
    top_teams = standings[:actual_count]
    team_count = actual_count
    bracket_size = next_power_of_2(team_count)  # e.g. 5→8, 8→8, 10→16
    first_round_name = get_first_round_name(team_count)
    num_byes = bracket_size - team_count  # top seeds get byes

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Clear existing preparation pairs
    cursor.execute("DELETE FROM tournament_games WHERE pair_id IN (SELECT id FROM tournament_pairs)")
    cursor.execute("DELETE FROM tournament_pairs")

    # Build seeding for standard single-elimination bracket.
    # Positions in the first round: pair seeds so 1 vs bracket_size, 2 vs bracket_size-1, etc.
    # Teams with seed <= num_byes get a BYE (no first-round match).
    # The rest play in the first round.
    #
    # E.g. 5 teams, bracket_size=8, num_byes=3:
    #   Seeds 1,2,3 get byes. Seeds 4,5 play in first round.
    #   Bracket: seed 1 vs BYE, seed 4 vs seed 5, seed 3 vs BYE, seed 2 vs BYE
    #   -> Only one actual match in first round: seed 4 vs seed 5
    #   -> Quarter-finals: 1 vs winner(4v5), 3 vs BYE-winner, 2 vs BYE-winner
    #
    # E.g. 8 teams, bracket_size=8, num_byes=0:
    #   All 8 teams play in first round: 1v8, 4v5, 3v6, 2v7

    # Generate ALL bracket slots (including BYEs)
    # BYE matches are stored as auto-completed pairs so advance_bracket works uniformly.
    half = bracket_size // 2
    real_match_count = 0
    for i in range(half):
        seed_a = i + 1  # 1-based seed
        seed_b = bracket_size - i  # mirror seed

        team_a = top_teams[seed_a - 1] if seed_a <= team_count else None
        team_b = top_teams[seed_b - 1] if seed_b <= team_count else None

        if team_a and team_b:
            # Real match
            cursor.execute("""
                INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """, (first_round_name, i, team_a["id"], team_b["id"], team_a["name"], team_b["name"], team_a["rank"], team_b["rank"]))
            pair_id = cursor.lastrowid
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 1, 'pending')", (pair_id,))
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, status) VALUES (?, 2, 'pending')", (pair_id,))
            real_match_count += 1

        elif team_a:
            # BYE: team_a auto-advances. Create pair with team_a as both teams (auto-win).
            cursor.execute("""
                INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed, winner_id)
                VALUES (?, ?, ?, NULL, ?, 'BYE', ?, NULL, ?)
            """, (first_round_name, i, team_a["id"], team_a["name"], team_a["rank"], team_a["id"]))
            pair_id = cursor.lastrowid
            # BYE games are auto-completed
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, team1_score, team2_score, status) VALUES (?, 1, 0, 0, 'completed')", (pair_id,))
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, team1_score, team2_score, status) VALUES (?, 2, 0, 0, 'completed')", (pair_id,))

        elif team_b:
            # BYE: team_b auto-advances
            cursor.execute("""
                INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed, winner_id)
                VALUES (?, ?, NULL, ?, 'BYE', ?, NULL, ?, ?)
            """, (first_round_name, i, team_b["id"], team_b["name"], team_b["rank"], team_b["id"]))
            pair_id = cursor.lastrowid
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, team1_score, team2_score, status) VALUES (?, 1, 0, 0, 'completed')", (pair_id,))
            cursor.execute("INSERT INTO tournament_games (pair_id, game_number, team1_score, team2_score, status) VALUES (?, 2, 0, 0, 'completed')", (pair_id,))

    set_tournament_phase("preparation", False, cursor=cursor)
    cursor.execute("UPDATE tournament_config SET team_count = ? WHERE id = 1", (team_count,))
    conn.commit()
    conn.close()

    return {
        "status": "generated",
        "team_count": team_count,
        "bracket_size": bracket_size,
        "first_round": first_round_name,
        "first_round_matches": real_match_count,
        "byes": num_byes,
    }


@app.get("/api/tournament/preparation")
async def get_tournament_preparation():
    """Get current tournament pairs for editing."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    # Get the first round name dynamically (the round with most pairs, excluding third_place/final)
    cursor.execute("""
        SELECT round FROM tournament_pairs 
        WHERE round NOT IN ('third_place', 'final')
        GROUP BY round ORDER BY COUNT(*) DESC LIMIT 1
    """)
    row = cursor.fetchone()
    first_round = row[0] if row else "quarter_final"

    cursor.execute("SELECT * FROM tournament_pairs WHERE round = ? ORDER BY match_index", (first_round,))
    pairs = [dict(r) for r in cursor.fetchall()]
    conn.close()

    standings = compute_qualification_standings()

    return {
        "pairs": pairs,
        "standings": standings,
        "phase": get_tournament_phase(),
        "first_round": first_round,
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
    """Add a new pair to the first round."""
    state = get_tournament_phase()
    if state["confirmed"]:
        raise HTTPException(status_code=400, detail="Tournament already confirmed")

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Detect first round dynamically
    cursor.execute("""
        SELECT round FROM tournament_pairs 
        WHERE round NOT IN ('third_place', 'final')
        GROUP BY round ORDER BY COUNT(*) DESC LIMIT 1
    """)
    row = cursor.fetchone()
    first_round = row[0] if row else "quarter_final"

    # Get next match index
    cursor.execute("SELECT COALESCE(MAX(match_index), -1) + 1 FROM tournament_pairs WHERE round = ?", (first_round,))
    next_idx = cursor.fetchone()[0]

    t1_name = teams[team1_id].name if team1_id in teams else f"Team {team1_id}"
    t2_name = teams[team2_id].name if team2_id in teams else f"Team {team2_id}"

    cursor.execute("""
        INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed)
        VALUES (?, ?, ?, ?, ?, ?, 0, 0)
    """, (first_round, next_idx, team1_id, team2_id, t1_name, t2_name))

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
    cursor.execute("SELECT COUNT(*) FROM tournament_pairs")
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

    # Get dynamic round info from current bracket state
    round_names, round_display, round_match_counts = get_bracket_round_names()

    bracket = {}
    for round_name in round_names:
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
            "display_name": round_display.get(round_name, round_name),
            "pairs": pairs,
        }

    conn.close()

    return {
        "state": get_tournament_phase(),
        "bracket": bracket,
        "round_names": round_names,
        "round_display": round_display,
        "round_match_counts": round_match_counts,
    }


def compute_duel_score(pair_dict, cursor=None):
    """Compute the tournament duel score including qualification advantage.

    Rules §2.3:
    - Each tournament goal = 1 point  (already includes foul-penalty goals in score cols)
    - Each qualification goal (from head-to-head between same teams) = 0.5 points
    - Tiebreaker: (1) total points, (2) qual duel winner, (3) better qual rank
    - Override: if override_team1_total / override_team2_total are set on the pair,
      those values replace the computed totals.
    """
    t1_id = pair_dict.get("team1_id")
    t2_id = pair_dict.get("team2_id")
    if not t1_id or not t2_id:
        return None

    games = pair_dict.get("games", [])
    scored_games = [g for g in games if g["status"] in ("completed", "active")]
    all_games = games  # include all games for foul tracking display

    # Tournament goals (1pt each) — score columns already include foul-penalty goals
    # Include both completed and active games for live score updates
    t1_tournament_goals = sum(g["team1_score"] for g in scored_games)
    t2_tournament_goals = sum(g["team2_score"] for g in scored_games)

    # Foul totals across all games (for display)
    t1_fouls_total = sum(g.get("team1_fouls", 0) for g in all_games)
    t2_fouls_total = sum(g.get("team2_fouls", 0) for g in all_games)

    # Foul penalty points: points awarded to opponent due to fouls
    # team1_foul_penalty = goals given to team2 because of team1's fouls
    # Since foul-penalty goals are already in the score columns, this is informational only
    t1_foul_penalty = sum(max(0, g.get("team1_fouls", 0) - 3) for g in all_games)
    t2_foul_penalty = sum(max(0, g.get("team2_fouls", 0) - 3) for g in all_games)

    # Qualification head-to-head goals (0.5pt each)
    t1_qual_goals, t2_qual_goals = get_head_to_head_goals(t1_id, t2_id)

    t1_total = t1_tournament_goals * 1.0 + t1_qual_goals * 0.5
    t2_total = t2_tournament_goals * 1.0 + t2_qual_goals * 0.5

    # Check for override
    override_t1 = pair_dict.get("override_team1_total")
    override_t2 = pair_dict.get("override_team2_total")
    override_active = override_t1 is not None and override_t2 is not None
    if override_active:
        t1_total = override_t1
        t2_total = override_t2

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
        # Foul tracking (informational)
        "team1_fouls_total": t1_fouls_total,
        "team2_fouls_total": t2_fouls_total,
        "team1_foul_points": t1_foul_penalty,  # points team1's fouls gave to team2
        "team2_foul_points": t2_foul_penalty,  # points team2's fouls gave to team1
        "override": override_active,
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


@app.post("/api/tournament/game/{game_id}/goal")
async def add_game_goal(game_id: int, action: GoalAction):
    """Increment goal score for team 1 or 2 in an active game."""
    if action.team not in (1, 2):
        raise HTTPException(status_code=400, detail="team must be 1 or 2")

    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    cursor.execute("SELECT * FROM tournament_games WHERE id = ?", (game_id,))
    row = cursor.fetchone()
    if not row:
        conn.close()
        raise HTTPException(status_code=404, detail="Game not found")
    game = dict(row)
    if game["status"] != "active":
        conn.close()
        raise HTTPException(status_code=400, detail="Game is not active")

    score_col = "team1_score" if action.team == 1 else "team2_score"
    cursor.execute(
        f"UPDATE tournament_games SET {score_col} = {score_col} + 1 WHERE id = ?",
        (game_id,)
    )
    conn.commit()

    cursor.execute("SELECT * FROM tournament_games WHERE id = ?", (game_id,))
    updated = dict(cursor.fetchone())
    conn.close()

    await broadcast_viz_update()
    return updated


@app.post("/api/tournament/game/{game_id}/foul")
async def add_game_foul(game_id: int, action: FoulAction):
    """Increment foul count for team 1 or 2 in an active game.
    The 4th and each subsequent foul awards 1 point to the opposing team.
    """
    if action.team not in (1, 2):
        raise HTTPException(status_code=400, detail="team must be 1 or 2")

    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    cursor.execute("SELECT * FROM tournament_games WHERE id = ?", (game_id,))
    row = cursor.fetchone()
    if not row:
        conn.close()
        raise HTTPException(status_code=404, detail="Game not found")
    game = dict(row)
    if game["status"] != "active":
        conn.close()
        raise HTTPException(status_code=400, detail="Game is not active")

    foul_col = "team1_fouls" if action.team == 1 else "team2_fouls"
    # Opposing team's score column
    opponent_score_col = "team2_score" if action.team == 1 else "team1_score"

    new_fouls = game[foul_col] + 1
    # 4th foul and every subsequent one awards a point to the opponent
    if new_fouls >= 4:
        cursor.execute(
            f"UPDATE tournament_games SET {foul_col} = ?, {opponent_score_col} = {opponent_score_col} + 1 WHERE id = ?",
            (new_fouls, game_id)
        )
    else:
        cursor.execute(
            f"UPDATE tournament_games SET {foul_col} = ? WHERE id = ?",
            (new_fouls, game_id)
        )
    conn.commit()

    cursor.execute("SELECT * FROM tournament_games WHERE id = ?", (game_id,))
    updated = dict(cursor.fetchone())
    conn.close()

    await broadcast_viz_update()
    return updated


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


@app.post("/api/tournament/pair/{pair_id}/override-score")
async def override_pair_score(pair_id: int, override: OverrideScore):
    """Override the total duel score for a pair (even if already completed).
    Re-determines the winner using the overridden scores.
    """
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    cursor.execute("SELECT * FROM tournament_pairs WHERE id = ?", (pair_id,))
    row = cursor.fetchone()
    if not row:
        conn.close()
        raise HTTPException(status_code=404, detail="Pair not found")
    pair = dict(row)

    # Set override totals
    cursor.execute(
        "UPDATE tournament_pairs SET override_team1_total = ?, override_team2_total = ? WHERE id = ?",
        (override.team1_total, override.team2_total, pair_id)
    )

    # Re-determine winner using the override scores
    t1_id = pair["team1_id"]
    t2_id = pair["team2_id"]
    if t1_id and t2_id:
        if override.team1_total > override.team2_total:
            winner_id = t1_id
        elif override.team2_total > override.team1_total:
            winner_id = t2_id
        else:
            # Tiebreaker: better seed (lower seed number = better rank)
            t1_seed = pair.get("team1_seed") or 999
            t2_seed = pair.get("team2_seed") or 999
            winner_id = t1_id if t1_seed <= t2_seed else t2_id
        cursor.execute("UPDATE tournament_pairs SET winner_id = ? WHERE id = ?", (winner_id, pair_id))

    conn.commit()

    # Return updated pair
    cursor.execute("SELECT * FROM tournament_pairs WHERE id = ?", (pair_id,))
    updated_pair = dict(cursor.fetchone())
    conn.close()

    await broadcast_viz_update()
    return updated_pair


@app.post("/api/tournament/pair/{pair_id}/clear-override")
async def clear_pair_override(pair_id: int):
    """Clear any score override on a pair and re-determine winner from normal scoring."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    cursor.execute("SELECT * FROM tournament_pairs WHERE id = ?", (pair_id,))
    row = cursor.fetchone()
    if not row:
        conn.close()
        raise HTTPException(status_code=404, detail="Pair not found")
    pair = dict(row)

    # Clear override
    cursor.execute(
        "UPDATE tournament_pairs SET override_team1_total = NULL, override_team2_total = NULL WHERE id = ?",
        (pair_id,)
    )

    # Re-determine winner from normal scoring if the pair was completed
    cursor.execute("SELECT * FROM tournament_games WHERE pair_id = ? ORDER BY game_number", (pair_id,))
    pair["games"] = [dict(g) for g in cursor.fetchall()]
    pair["override_team1_total"] = None
    pair["override_team2_total"] = None

    all_completed = all(g["status"] == "completed" for g in pair["games"]) and len(pair["games"]) > 0
    if all_completed and pair.get("team1_id") and pair.get("team2_id"):
        duel = compute_duel_score(pair, cursor)
        winner_id = determine_winner(pair, duel)
        cursor.execute("UPDATE tournament_pairs SET winner_id = ? WHERE id = ?", (winner_id, pair_id))

    conn.commit()

    cursor.execute("SELECT * FROM tournament_pairs WHERE id = ?", (pair_id,))
    updated_pair = dict(cursor.fetchone())
    conn.close()

    await broadcast_viz_update()
    return updated_pair


def determine_winner(pair_dict, duel_score):
    """Determine the winner of a tournament pair per rules §2.3.
    If override totals are active (duel_score['override'] is True),
    the overridden total_points are already applied in duel_score.
    """
    if not duel_score:
        return None

    t1_id = pair_dict["team1_id"]
    t2_id = pair_dict["team2_id"]

    # (1) Total points (already reflects override if active)
    if duel_score["team1_total_points"] > duel_score["team2_total_points"]:
        return t1_id
    elif duel_score["team2_total_points"] > duel_score["team1_total_points"]:
        return t2_id

    # If override is active and scores are tied, use seed as tiebreaker directly
    if duel_score.get("override"):
        t1_seed = duel_score.get("team1_seed") or 999
        t2_seed = duel_score.get("team2_seed") or 999
        return t1_id if t1_seed <= t2_seed else t2_id

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
    """Generate the next round's pairs from the completed round's winners.
    
    Uses dynamic round progression based on current bracket size.
    Handles byes: if a next-round pair already exists with one known team
    and one pending slot, fills in the winner.
    """
    # Build dynamic round progression from DB state
    round_names, _, _ = get_bracket_round_names()

    # Find next round in sequence (excluding third_place/final from normal progression)
    competitive_rounds = [r for r in round_names if r not in ("third_place", "final")]

    if completed_round == "semi_final":
        # Special: generates both third_place and final
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

        # Create third place match (skip if any loser is a BYE/NULL team)
        real_losers = [l for l in losers if l["id"] is not None]
        if len(real_losers) >= 2:
            cursor.execute("""
                INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed)
                VALUES ('third_place', 0, ?, ?, ?, ?, ?, ?)
            """, (real_losers[0]["id"], real_losers[1]["id"], real_losers[0]["name"], real_losers[1]["name"], real_losers[0]["seed"], real_losers[1]["seed"]))
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

    # Find the next round in competitive_rounds
    if completed_round not in competitive_rounds:
        return
    idx = competitive_rounds.index(completed_round)
    if idx + 1 >= len(competitive_rounds):
        return  # No more rounds (shouldn't happen, semi_final handled above)
    next_round = competitive_rounds[idx + 1]

    def get_winner_info(p):
        wid = p["winner_id"]
        if wid is not None and wid == p.get("team1_id"):
            return {"id": p["team1_id"], "name": p["team1_name"], "seed": p["team1_seed"]}
        if wid is not None and wid == p.get("team2_id"):
            return {"id": p["team2_id"], "name": p["team2_name"], "seed": p["team2_seed"]}
        # Fallback — shouldn't happen if data is consistent
        if p.get("team1_id") is not None:
            return {"id": p["team1_id"], "name": p["team1_name"], "seed": p["team1_seed"]}
        return {"id": p["team2_id"], "name": p["team2_name"], "seed": p["team2_seed"]}

    # Get winners from completed round in order
    cursor.execute("""
        SELECT * FROM tournament_pairs WHERE round = ? ORDER BY match_index
    """, (completed_round,))
    pairs = [dict(r) for r in cursor.fetchall()]

    # Check if next-round pairs already exist (from bye pre-population)
    cursor.execute("""
        SELECT * FROM tournament_pairs WHERE round = ? ORDER BY match_index
    """, (next_round,))
    existing_next = {dict(r)["match_index"]: dict(r) for r in cursor.fetchall()}

    # Winners pair up: pair 0+1 -> next match 0, pair 2+3 -> next match 1, etc.
    # But we need to map by match_index, not by position in the result set
    # The match_index in the current round maps to next round: 
    #   current match_index i feeds into next round match_index i//2
    # Group completed-round pairs by their next-round match slot
    next_round_feeds = defaultdict(list)  # next_match_idx -> list of winner info
    for p in pairs:
        w = get_winner_info(p)
        next_match_idx = p["match_index"] // 2
        next_round_feeds[next_match_idx].append(w)

    for next_match_idx, winners in next_round_feeds.items():
        if next_match_idx in existing_next:
            # Next-round pair already exists (pre-populated for byes).
            # Fill in any NULL team slots with the winner(s).
            existing = existing_next[next_match_idx]
            for w in winners:
                if existing["team1_id"] is None:
                    cursor.execute("""
                        UPDATE tournament_pairs SET team1_id = ?, team1_name = ?, team1_seed = ?
                        WHERE id = ?
                    """, (w["id"], w["name"], w["seed"], existing["id"]))
                    existing["team1_id"] = w["id"]  # update local copy
                elif existing["team2_id"] is None:
                    cursor.execute("""
                        UPDATE tournament_pairs SET team2_id = ?, team2_name = ?, team2_seed = ?
                        WHERE id = ?
                    """, (w["id"], w["name"], w["seed"], existing["id"]))
                    existing["team2_id"] = w["id"]
            continue

        if len(winners) < 2:
            continue

        w1, w2 = winners[0], winners[1]

        cursor.execute("""
            INSERT INTO tournament_pairs (round, match_index, team1_id, team2_id, team1_name, team2_name, team1_seed, team2_seed)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """, (next_round, next_match_idx, w1["id"], w2["id"], w1["name"], w2["name"], w1["seed"], w2["seed"]))

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

@app.get("/teams")
@app.get("/teams.html")
async def serve_teams():
    return FileResponse(os.path.join(WWW_DIR, "teams.html"))

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
