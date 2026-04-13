"""
competition_proxy.py — Embedded TCP reverse proxy for FuzbAI competition

Pure Python asyncio, no external dependencies. Provides transparent HTTP
byte-forwarding between team ports and MuJoCo simulator ports.

Usage (embedded in competition manager):

    from competition_proxy import ProxyManager

    proxy = ProxyManager()

    # On FastAPI startup:
    async def startup():
        await proxy.add_team(1, listen_port=24001)
        await proxy.add_team(2, listen_port=24002)
        ...

    # When assigning a match pairing:
    proxy.set_route(team_id=1, upstream_host='127.0.0.1', upstream_port=25001)
    proxy.set_route(team_id=2, upstream_host='127.0.0.1', upstream_port=25002)

    # When the match ends:
    proxy.clear_route(team_id=1)
    proxy.clear_route(team_id=2)

    # Query statistics:
    stats = proxy.get_stats(team_id=1)
"""

import asyncio
import collections
import time
import logging

logger = logging.getLogger("competition_proxy")

# ---------------------------------------------------------------------------
# HTTP 503 response sent when a team is not assigned to any simulator
# ---------------------------------------------------------------------------
_503_RESPONSE = (
    b"HTTP/1.1 503 Service Unavailable\r\n"
    b"Content-Type: text/plain\r\n"
    b"Content-Length: 35\r\n"
    b"Connection: close\r\n"
    b"\r\n"
    b"Team not assigned to a simulator.\r\n"
)

# Buffer size for TCP forwarding
_BUF_SIZE = 16384


class ProxyStats:
    """Per-team connection statistics."""

    _RATE_WINDOW = 5.0  # rolling window in seconds for req/s calculation

    __slots__ = (
        "connections_total",
        "connections_active",
        "last_activity",
        "rejected_unassigned",
        "_request_times",
    )

    def __init__(self):
        self.connections_total: int = 0
        self.connections_active: int = 0
        self.last_activity: float = 0.0
        self.rejected_unassigned: int = 0
        self._request_times: collections.deque = collections.deque()

    def record_transfer(self):
        """Record one upstream data read for req/s rate tracking."""
        now = time.time()
        self._request_times.append(now)
        cutoff = now - self._RATE_WINDOW
        while self._request_times and self._request_times[0] < cutoff:
            self._request_times.popleft()
        self.last_activity = now

    @property
    def requests_per_second(self) -> float:
        """Rolling requests-per-second over the last window."""
        now = time.time()
        cutoff = now - self._RATE_WINDOW
        while self._request_times and self._request_times[0] < cutoff:
            self._request_times.popleft()
        return len(self._request_times) / self._RATE_WINDOW

    def to_dict(self) -> dict:
        return {
            "connections_total": self.connections_total,
            "connections_active": self.connections_active,
            "last_activity": self.last_activity,
            "idle_seconds": round(time.time() - self.last_activity, 1) if self.last_activity else None,
            "rejected_unassigned": self.rejected_unassigned,
            "requests_per_second": round(self.requests_per_second, 1),
        }


class TeamProxy:
    """
    Manages a single team's TCP proxy listener.

    Accepts connections on ``listen_port`` and transparently forwards bytes
    to/from the assigned upstream (simulator) port.
    """

    def __init__(self, team_id: int, listen_port: int, listen_host: str = "0.0.0.0"):
        self.team_id = team_id
        self.listen_port = listen_port
        self.listen_host = listen_host

        self.upstream_host: str | None = None
        self.upstream_port: int | None = None

        self.stats = ProxyStats()

        self._server: asyncio.Server | None = None
        self._active_tasks: set[asyncio.Task] = set()

    # -- lifecycle -----------------------------------------------------------

    async def start(self):
        """Start listening for connections on the team port."""
        self._server = await asyncio.start_server(
            self._handle_client, self.listen_host, self.listen_port
        )
        logger.info(
            "Team %d proxy listening on %s:%d",
            self.team_id, self.listen_host, self.listen_port,
        )

    async def stop(self):
        """Stop the listener and cancel all active connections."""
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        # Cancel any in-flight forwarding tasks
        for task in self._active_tasks:
            task.cancel()
        self._active_tasks.clear()
        logger.info("Team %d proxy stopped", self.team_id)

    # -- routing -------------------------------------------------------------

    def set_upstream(self, host: str, port: int):
        """Assign this team to a simulator port."""
        self.upstream_host = host
        self.upstream_port = port
        logger.info(
            "Team %d routed to %s:%d", self.team_id, host, port,
        )

    def clear_upstream(self):
        """Unassign this team (new connections will get 503)."""
        self.upstream_host = None
        self.upstream_port = None
        logger.info("Team %d route cleared", self.team_id)

    async def drop_connections(self):
        """Force-close all active connections (e.g. between matches)."""
        for task in self._active_tasks:
            task.cancel()
        self._active_tasks.clear()

    # -- connection handling -------------------------------------------------

    async def _handle_client(self, client_reader: asyncio.StreamReader,
                             client_writer: asyncio.StreamWriter):
        """Called for each new TCP connection from a team."""
        task = asyncio.current_task()
        self._active_tasks.add(task)

        self.stats.connections_total += 1
        self.stats.connections_active += 1

        peer = client_writer.get_extra_info("peername")
        logger.debug("Team %d: connection from %s", self.team_id, peer)

        upstream_writer: asyncio.StreamWriter | None = None

        try:
            # ----- unassigned: send 503 and close ---------------------------
            if self.upstream_port is None or self.upstream_host is None:
                self.stats.rejected_unassigned += 1
                # Read enough to consume the HTTP request before responding
                try:
                    await asyncio.wait_for(client_reader.read(_BUF_SIZE), timeout=1.0)
                except asyncio.TimeoutError:
                    pass
                client_writer.write(_503_RESPONSE)
                await client_writer.drain()
                return

            # ----- connect to the simulator ---------------------------------
            try:
                upstream_reader, upstream_writer = await asyncio.wait_for(
                    asyncio.open_connection(self.upstream_host, self.upstream_port),
                    timeout=5.0,
                )
            except (OSError, asyncio.TimeoutError) as exc:
                logger.warning(
                    "Team %d: cannot reach upstream %s:%d (%s)",
                    self.team_id, self.upstream_host, self.upstream_port, exc,
                )
                # Consume the request, send 502
                try:
                    await asyncio.wait_for(client_reader.read(_BUF_SIZE), timeout=1.0)
                except asyncio.TimeoutError:
                    pass
                client_writer.write(
                    b"HTTP/1.1 502 Bad Gateway\r\n"
                    b"Content-Type: text/plain\r\n"
                    b"Content-Length: 28\r\n"
                    b"Connection: close\r\n"
                    b"\r\n"
                    b"Simulator not reachable.\r\n\r\n"
                )
                await client_writer.drain()
                return

            # ----- bidirectional byte forwarding ----------------------------
            up_task = asyncio.ensure_future(
                self._pipe(client_reader, upstream_writer, upstream=True)
            )
            down_task = asyncio.ensure_future(
                self._pipe(upstream_reader, client_writer, upstream=False)
            )

            _done, pending = await asyncio.wait(
                [up_task, down_task], return_when=asyncio.FIRST_COMPLETED,
            )
            for t in pending:
                t.cancel()
                try:
                    await t
                except asyncio.CancelledError:
                    pass

        except asyncio.CancelledError:
            pass
        except Exception:
            logger.debug("Team %d: connection error", self.team_id, exc_info=True)
        finally:
            self.stats.connections_active -= 1
            self._active_tasks.discard(task)
            _safe_close(client_writer)
            if upstream_writer is not None:
                _safe_close(upstream_writer)

    async def _pipe(self, reader: asyncio.StreamReader,
                    writer: asyncio.StreamWriter, *, upstream: bool):
        """Copy bytes from *reader* to *writer* until EOF or error.

        When *upstream* is True (client → simulator), each successful read is
        counted toward the rolling req/s rate so the UI can reflect whether the
        client is actively sending data.
        """
        try:
            while True:
                data = await reader.read(_BUF_SIZE)
                if not data:
                    break
                if upstream:
                    self.stats.record_transfer()
                else:
                    self.stats.last_activity = time.time()
                writer.write(data)
                await writer.drain()
        except (ConnectionResetError, BrokenPipeError,
                ConnectionAbortedError, asyncio.CancelledError):
            pass


def _safe_close(writer: asyncio.StreamWriter):
    """Close a StreamWriter, ignoring errors."""
    try:
        if not writer.is_closing():
            writer.close()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# ProxyManager — the public API for the competition manager
# ---------------------------------------------------------------------------

class ProxyManager:
    """
    Manages proxy listeners for all teams.

    Typical lifecycle::

        mgr = ProxyManager()
        await mgr.add_team(1, 24001)
        await mgr.add_team(2, 24002)

        mgr.set_route(1, '127.0.0.1', 25001)   # team 1 → sim red
        mgr.set_route(2, '127.0.0.1', 25002)   # team 2 → sim blue

        # ... match plays ...

        mgr.clear_route(1)
        mgr.clear_route(2)
        await mgr.stop_all()
    """

    def __init__(self):
        self.teams: dict[int, TeamProxy] = {}

    async def add_team(self, team_id: int, listen_port: int,
                       listen_host: str = "0.0.0.0") -> TeamProxy:
        """Register a team and start its proxy listener."""
        if team_id in self.teams:
            logger.warning("Team %d already registered, skipping", team_id)
            return self.teams[team_id]
        proxy = TeamProxy(team_id, listen_port, listen_host)
        await proxy.start()
        self.teams[team_id] = proxy
        return proxy

    async def add_teams(self, count: int, base_port: int,
                        listen_host: str = "0.0.0.0"):
        """Convenience: register *count* teams with consecutive ports."""
        for i in range(1, count + 1):
            await self.add_team(i, base_port + i, listen_host)

    async def remove_team(self, team_id: int):
        """Stop and unregister a team's proxy."""
        if team_id in self.teams:
            await self.teams[team_id].stop()
            del self.teams[team_id]

    def set_route(self, team_id: int, upstream_host: str, upstream_port: int):
        """Point a team's proxy at a simulator port."""
        if team_id in self.teams:
            self.teams[team_id].set_upstream(upstream_host, upstream_port)
        else:
            logger.warning("set_route: team %d not registered", team_id)

    def clear_route(self, team_id: int):
        """Unassign a team (future connections get HTTP 503)."""
        if team_id in self.teams:
            self.teams[team_id].clear_upstream()

    async def drop_connections(self, team_id: int):
        """Force-close all active connections for a team."""
        if team_id in self.teams:
            await self.teams[team_id].drop_connections()

    def get_stats(self, team_id: int) -> dict | None:
        """Return connection statistics for a team."""
        if team_id in self.teams:
            return self.teams[team_id].stats.to_dict()
        return None

    def get_all_stats(self) -> dict[int, dict]:
        """Return statistics for all teams."""
        return {tid: tp.stats.to_dict() for tid, tp in self.teams.items()}

    async def stop_all(self):
        """Stop all team proxies."""
        for team_id in list(self.teams.keys()):
            await self.remove_team(team_id)


# ---------------------------------------------------------------------------
# Standalone demo / self-test
# ---------------------------------------------------------------------------

async def _demo():
    """Quick smoke test: start a proxy, show that it listens."""
    logging.basicConfig(level=logging.INFO, format="%(name)s | %(message)s")

    mgr = ProxyManager()
    await mgr.add_teams(count=4, base_port=24000)

    # Simulate assigning two matches
    mgr.set_route(1, "127.0.0.1", 25001)
    mgr.set_route(2, "127.0.0.1", 25002)
    # Teams 3 & 4 stay unassigned → will get 503

    print("\nProxy is running. Try:")
    print("  curl http://127.0.0.1:24001/Camera/State   → forwarded to 25001")
    print("  curl http://127.0.0.1:24003/Camera/State   → returns 503")
    print("\nPress Ctrl+C to stop.\n")

    try:
        await asyncio.Event().wait()  # run forever
    except asyncio.CancelledError:
        pass
    finally:
        await mgr.stop_all()


if __name__ == "__main__":
    try:
        asyncio.run(_demo())
    except KeyboardInterrupt:
        print("\nStopped.")
