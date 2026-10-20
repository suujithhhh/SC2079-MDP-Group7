# Bluetooth Test Strings — Android Remote Controller Checklist

Test strings for the `Android Remote Controller checklist.pdf` (items C.1–C.10), matched
against the actual implementation in `protocol/ProtocolParser.kt` and
`protocol/ProtocolSerializer.kt`. Use these with the AMD tool to simulate messages from
the robot/RPi, and observe what the app sends for outbound actions.

## The crucial one: Target ID recognition (C.9)

Send from AMD tool → Android:

```
TARGET,B2,11
```

Updates obstacle `B2`'s block to display target ID `11` and updates the UI. Format is
`TARGET,<obstacleId>,<targetId>` — `<obstacleId>` accepts `B2` or bare `2`; `<targetId>`
must be one of a whitelisted set of ~30 valid image IDs (11–19, 20–35, 36–40) or the
line is silently dropped.

4-field variant that also marks the target face:

```
TARGET,B2,11,N
```

## C.1 / C.4 — bidirectional text transfer & status TextView

Send from AMD tool → Android:

```
MSG,[Looking for target 2]
```

Note: the verb is `MSG`, not `STATUS`. The payload must be wrapped in literal `[...]`
brackets — the parser strips them before displaying the text.

## C.2 / C.8 — Bluetooth scan/connect/reconnect

No wire string needed. Test via the Connect/Disconnect UI flow, and the AMD tool's
Connect/Disconnect actions (to verify auto-reconnect for C.8).

## C.3 — movement control (Android → robot, via UI buttons only — not typed)

Verify the app emits these when you tap the movement controls:

```
MOVE,F
MOVE,B
TURN,FL
TURN,FR
TURN,BL
TURN,BR
```

## C.6 — obstacle placement (Android → robot)

```
ADD,B1,(10,6)
SUB,B1
```

## C.7 — obstacle face annotation (Android → robot)

```
FACE,B2,N
```

## C.10 — robot position/direction update (robot → Android)

Send from AMD tool → Android:

```
ROBOT,7,2,N
```

Exactly 3 comma-separated fields: integer x, integer y, single-char direction
(N/S/E/W, case-insensitive).

## Notes

- All verbs are case-insensitive and tolerate stray whitespace around fields
  (via `.trim()`), but canonical form has **no spaces after commas** — send exactly
  as shown above.
- Unknown verbs, wrong field counts, or invalid values are silently dropped
  (parser returns `null`) — the app won't crash, but also won't show an error. If
  nothing updates, double-check the test string against the formats above.
- `MOVE`/`TURN` vocabulary is marked `[ours]` in `docs/design/protocol-spec.md` —
  invented by this team, not yet confirmed with the robot/algorithm sub-team.
