"""
test_proxy.py — Automated test for competition_proxy.py

Starts a mock simulator (simple HTTP echo server), starts the proxy,
and verifies:
  1. Assigned team: requests are forwarded correctly
  2. Unassigned team: gets HTTP 503
  3. Unreachable upstream: gets HTTP 502
  4. Statistics are tracked
"""

import asyncio
import time

# ---------------------------------------------------------------------------
# Mock simulator: minimal HTTP server that echoes request info as JSON
# ---------------------------------------------------------------------------

async def mock_sim_handler(reader: asyncio.StreamReader,
                           writer: asyncio.StreamWriter):
    """Handle one HTTP connection from the proxy."""
    try:
        # Read the HTTP request (we don't need to fully parse it)
        request_data = await asyncio.wait_for(reader.read(8192), timeout=5.0)
        if not request_data:
            return

        # Extract the request line
        request_line = request_data.split(b"\r\n", 1)[0].decode()

        # Build a simple JSON response body
        body = f'{{"echo": "{request_line}", "time": {time.time():.3f}}}'
        body_bytes = body.encode()

        response = (
            b"HTTP/1.1 200 OK\r\n"
            b"Content-Type: application/json\r\n"
            b"Content-Length: " + str(len(body_bytes)).encode() + b"\r\n"
            b"Connection: close\r\n"
            b"\r\n"
            + body_bytes
        )
        writer.write(response)
        await writer.drain()
    except Exception as e:
        print(f"  [mock sim] error: {e}")
    finally:
        try:
            writer.close()
        except Exception:
            pass


async def make_http_request(host: str, port: int, path: str = "/Camera/State") -> tuple[int, str]:
    """Make a raw HTTP GET request and return (status_code, body)."""
    reader, writer = await asyncio.open_connection(host, port)
    request = f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nConnection: close\r\n\r\n"
    writer.write(request.encode())
    await writer.drain()

    response = b""
    while True:
        chunk = await reader.read(4096)
        if not chunk:
            break
        response += chunk

    writer.close()

    # Parse status code and body
    header_end = response.find(b"\r\n\r\n")
    status_line = response[:response.find(b"\r\n")].decode()
    status_code = int(status_line.split(" ", 2)[1])
    body = response[header_end + 4:].decode() if header_end >= 0 else ""

    return status_code, body


async def run_tests():
    from competition_proxy import ProxyManager

    MOCK_SIM_PORT = 29901
    TEAM1_PORT = 29801
    TEAM2_PORT = 29802
    TEAM3_PORT = 29803

    passed = 0
    failed = 0

    def check(name, condition):
        nonlocal passed, failed
        if condition:
            print(f"  PASS: {name}")
            passed += 1
        else:
            print(f"  FAIL: {name}")
            failed += 1

    # --- Start mock simulator -----------------------------------------------
    print("Starting mock simulator on port", MOCK_SIM_PORT)
    sim_server = await asyncio.start_server(mock_sim_handler, "127.0.0.1", MOCK_SIM_PORT)

    # --- Start proxy --------------------------------------------------------
    print("Starting proxy...")
    mgr = ProxyManager()
    await mgr.add_team(1, TEAM1_PORT, "127.0.0.1")
    await mgr.add_team(2, TEAM2_PORT, "127.0.0.1")
    await mgr.add_team(3, TEAM3_PORT, "127.0.0.1")

    # Team 1 → mock simulator
    mgr.set_route(1, "127.0.0.1", MOCK_SIM_PORT)
    # Team 2 stays unassigned
    # Team 3 → unreachable port
    mgr.set_route(3, "127.0.0.1", 29999)

    await asyncio.sleep(0.1)  # let servers start

    # --- Test 1: Forwarded request ------------------------------------------
    print("\nTest 1: Assigned team (forwarded request)")
    status, body = await make_http_request("127.0.0.1", TEAM1_PORT, "/Camera/State")
    check("Status is 200", status == 200)
    check("Body contains echo", "Camera/State" in body)
    check("Body is JSON-like", body.startswith("{"))

    # --- Test 2: Unassigned team → 503 --------------------------------------
    print("\nTest 2: Unassigned team (503)")
    status, body = await make_http_request("127.0.0.1", TEAM2_PORT, "/Camera/State")
    check("Status is 503", status == 503)
    check("Body mentions unassigned", "not assigned" in body.lower())

    # --- Test 3: Unreachable upstream → 502 ---------------------------------
    print("\nTest 3: Unreachable upstream (502)")
    status, body = await make_http_request("127.0.0.1", TEAM3_PORT, "/Camera/State")
    check("Status is 502", status == 502)

    # --- Test 4: Statistics -------------------------------------------------
    print("\nTest 4: Statistics")
    stats1 = mgr.get_stats(1)
    stats2 = mgr.get_stats(2)
    check("Team 1 has connections", stats1["connections_total"] >= 1)
    check("Team 1 has req/s stat", "requests_per_second" in stats1)
    check("Team 1 req/s is non-negative", stats1["requests_per_second"] >= 0)
    check("Team 2 has rejected count", stats2["rejected_unassigned"] >= 1)

    # --- Test 5: Route change -----------------------------------------------
    print("\nTest 5: Dynamic route change")
    mgr.set_route(2, "127.0.0.1", MOCK_SIM_PORT)
    await asyncio.sleep(0.05)
    status, body = await make_http_request("127.0.0.1", TEAM2_PORT, "/Motors/SendCommand")
    check("Previously-503 team now gets 200", status == 200)
    check("Request path forwarded correctly", "Motors/SendCommand" in body)

    # --- Test 6: Clear route ------------------------------------------------
    print("\nTest 6: Clear route")
    mgr.clear_route(2)
    await asyncio.sleep(0.05)
    status, _ = await make_http_request("127.0.0.1", TEAM2_PORT, "/Camera/State")
    check("Cleared team gets 503 again", status == 503)

    # --- Test 7: All stats --------------------------------------------------
    print("\nTest 7: All stats summary")
    all_stats = mgr.get_all_stats()
    check("All 3 teams have stats", len(all_stats) == 3)

    # --- Cleanup ------------------------------------------------------------
    print("\nCleaning up...")
    await mgr.stop_all()
    sim_server.close()
    await sim_server.wait_closed()

    print(f"\n{'='*40}")
    print(f"Results: {passed} passed, {failed} failed")
    print(f"{'='*40}")

    return failed == 0


if __name__ == "__main__":
    import logging
    logging.basicConfig(level=logging.INFO, format="%(name)s | %(message)s")
    success = asyncio.run(run_tests())
    exit(0 if success else 1)
