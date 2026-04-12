# Settings Lock Plugin

Password-protect grblHAL settings over serial/USB. Prevents accidental or unauthorized changes to machine configuration.

## Commands

| Command | Description |
|---|---|
| `$SETPWD=<password>` | Set or change the lock password (max 32 chars). Use `$SETPWD=` (empty) to clear. |
| `$LOCK` | Lock settings immediately. |
| `$UNLOCK=<password>` | Unlock settings for the current session. |

## Behavior

- When locked, any `$<number>=<value>` write or `$RST=` reset is rejected with `error:53`.
- Settings can still be **read** while locked (`$$`, `$ES`, etc.).
- Lock re-engages automatically on every soft reset / power cycle.
- Password is stored in NVS and survives power cycles.
- No password set = lock disabled (all writes allowed).

## Typical Workflow

```
$SETPWD=mypass    → Password saved.
$LOCK             → Settings locked.
$100=800          → error:53 (blocked)
$UNLOCK=mypass    → Settings unlocked.
$100=800          → ok
$LOCK             → Settings locked again.
```

## Status Report

The plugin reports its state in `$I+` output:

- `[SETLOCK:DISABLED - no password set]` — no password configured
- `[SETLOCK:LOCKED]` — active and locked
- `[SETLOCK:UNLOCKED]` — active, temporarily unlocked
