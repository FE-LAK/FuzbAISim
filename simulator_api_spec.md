# FuzbAI Simulator API Specification

To be compatible with the **FuzbAI Competition Manager v2**, each simulator must implement a management API on its designated `port_mgmt`. The Manager expects the following endpoints:

## 1. Get Status
**Endpoint**: `GET /status`

Returns the current tactical state of the simulator.

### Response Body (JSON)
```json
{
  "status": "string",   // e.g., "running", "paused", "waiting"
  "score": [int, int],  // [Red Team Score, Blue Team Score]
  "gametime": float     // Elapsed match time in seconds
}
```

## 2. Match Control
The Manager sends simple POST requests to trigger match state transitions.

### Start Match
**Endpoint**: `POST /start`
**Response**: `{"status": "ok"}` (or similar JSON)

### Pause Match
**Endpoint**: `POST /pause`
**Response**: `{"status": "ok"}`

### Reset Match
**Endpoint**: `POST /reset`
**Response**: `{"status": "ok"}`

## 3. Set Team Names
**Endpoint**: `POST /teams`

Sets the display names for the teams currently assigned to the simulator. This is typically called by the Manager during match assignment.

### Request Body (JSON)
```json
{
  "red": "string",  // Name of the Red team
  "blue": "string"  // Name of the Blue team
}
```

### Response
**Response**: `{"status": "ok"}`

---

## Technical Notes
- **Payloads**: All endpoints should ideally use `application/json` content type.
- **Timeouts**: The Manager typically expects a response within **2.0 seconds** (except for status checks which use a **0.2s** timeout for rapid UI updates).
- **CORS**: If the Manager's GUI were to call these directly (unlikely as it's proxied through the backend), CORS headers would be needed. However, the current backend-to-simulator architecture does not require it.
