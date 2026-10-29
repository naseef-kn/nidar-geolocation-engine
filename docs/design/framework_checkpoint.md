# Framework checkpoint: concurrent scout and delivery with reassignment

## Goal

Both drones (alpha, bravo) run scouting and delivery at the same time.
Any drone can detect targets and deliver to targets. When a drone is
delivering to a low-confidence target and a high-confidence target is found
elsewhere, the coordinator may reassign that drone to the better target.

## Roles

- Each drone has two independent slots:
  - Scout feed: reports detections as `target_found`.
  - Delivery slot: holds at most one active target at a time.
- Neither slot blocks the other.
- Capabilities in `config/*.json` are both `scout` and `delivery`.

## Ownership

- The coordinator runs at `gcs`. It holds the target registry.
- Drones report facts (detections, delivery progress) and execute assignments.
- Drones never decide on their own to switch targets.

## Target lifecycle

```text
DETECTED --assign--> ASSIGNED --drone confirms start--> IN_DELIVERY --> DELIVERED
                                                                |
                                                  reassignment  v
    FLAGGED <--------------------------------------------- (switched away)
       |
       +--re-verified by a scout--> DETECTED
```

- DETECTED: found by a scout, not assigned.
- ASSIGNED: a drone has been told to deliver, not yet confirmed started.
- IN_DELIVERY: the drone confirmed it has started.
- FLAGGED: delivery stopped because a better target was found. Kept for re-verification.
- DELIVERED: delivery complete.

## Reassignment rule

A switch from current target C to candidate T happens only if all hold:

1. C.status is IN_DELIVERY and C.assigned_to is set.
2. T.status is DETECTED.
3. C.confidence < low_confidence_below.
4. T.confidence >= high_confidence_at.
5. T.confidence - C.confidence >= min_margin.

On switch, C becomes FLAGGED with no drone, and T becomes ASSIGNED to the same drone.

Thresholds are configuration, not constants. Placeholder values are
0.6 / 0.85 / 0.2 and must be tuned from field data.

## Scaling notes

- The rule takes any number of drones and targets. It does not assume two drones.
- The policy is a separate object. A different policy, or a learned one, can
  replace it without changing the registry or the message types.
- Registry operations are keyed by target ID, so the same code works for any fleet size.

## Open questions (decide before flight)

1. Drop phase: should a drone in the final drop phase ever be switched? Current
   answer: no. The status would need a `DROPPING` state, which is not added yet.
2. Stale reports: a detection reported seconds ago may no longer be valid. Message
   timestamps and an expiry rule are needed before the coordinator acts on them.
3. Flagged target re-verification: who goes back, and when? Not decided.
4. Exhaustion: what happens when a drone has no unassigned target left? Not decided.

## Checkpoint plan

1. (this step) Target model, registry, reassignment rule, tests. No messaging.
2. Coordinator: consumes `target_found` and delivery messages, applies the rule,
   sends assignment and switch messages through the transport.
3. Drone task slots and new message types (`delivery_flagged`, `delivery_switched`),
   with capability checks.
4. Two-drone simulation script running both drones concurrently in one process.
