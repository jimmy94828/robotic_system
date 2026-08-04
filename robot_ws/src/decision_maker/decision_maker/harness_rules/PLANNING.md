# Planning Rules

This document describes high-level task decomposition rules for the agent-based decision maker.

## Closed-Loop Planning

- Produce exactly one next capability call per decision.
- Do not produce a full multi-step plan in iterative mode.
- Use the latest execution observation before choosing the next capability.
- Prefer symbolic targets such as `table`, `cabinet`, or `pringles`; do not invent low-level robot controls.


## Direct Grasp Tasks

For tasks like `grasp the bottle`, `grab bottle`, or `pick up bottle` with no source or destination:

1. Call `grasp_place` with `action=grasp` and the object target.
2. Do not run `object_query` or `navigation` first; the object is expected to be in the current camera view.
3. Finish after the grasp action succeeds.

## Direct Grasp-And-Place Tasks

For `grasp <object> and place it on <destination>`:

1. Call `grasp_place(action=grasp)` immediately; its pre-action view check handles view-angle adjustment.
2. Do not query or navigate to the object/source first.
3. After grasp succeeds, call `object_query` for the destination.
4. Navigate to the destination.
5. Call `grasp_place(action=place)` with the object and destination, then finish.

## Transfer Tasks

For tasks like `bring <object> on <source> to <destination>`:

1. First try `object_query` on the object itself.
2. If the object lookup succeeds, navigate to that object pose.
3. If the object lookup fails, use the failure observation to query the source location.
4. Navigate to the source only after the source lookup succeeds.
5. Grasp only the object, not the combined phrase.
6. Locate the destination with `object_query`.
7. Navigate to the destination.
8. Place the object at the destination.
9. Finish only after place succeeds.

The final action for `bring`, `move`, `take`, or `carry` to a place, surface, or furniture is always `grasp_place(action=place)`. Use `action=handover` only when the task explicitly says `handover`, `hand over`, `give`, or `pass`, or identifies a human receiver.

For an explicit handover task, follow the same query, navigation, and grasp sequence, then use `grasp_place(action=handover)` after reaching the receiver location.

For `hand it to the person sitting/seated on <location>`, the destination is the location. Query and navigate to that location; never query `person`.

For tasks like `move <object> from <source> to <destination>`:

1. First try `object_query` on the object itself.
2. If the object lookup succeeds, navigate to that object pose.
3. If the object lookup fails, use the failure observation to query the source location.
4. Navigate to the source only after the source lookup succeeds.
5. Grasp only the object, then locate/navigate to the destination and place it.

Example:

```json
{"capability":"object_query","target":"table","reason":"Need to locate the source."}
```

## Multi-Object Transfer Tasks

For tasks with multiple objects joined by `and` or commas, the LLM planner must reason about the object list and decompose the request into one complete transfer per object. Never use the combined object phrase as a target.

Example task:

```text
bring bottle and apple to table
```

Expected LLM reasoning/decomposition:

1. Locate `bottle`.
2. Navigate to `bottle`.
3. Grasp `bottle`.
4. Locate `table`.
5. Navigate to `table`.
6. Place `bottle` at `table`.
7. Locate `apple`.
8. Navigate to `apple`.
9. Grasp `apple`.
10. Locate `table`.
11. Navigate to `table`.
12. Place `apple` at `table`.
13. Finish only after every object has been placed.

For tasks where multiple objects share a source, such as:

```text
bring bottle and apple on cabinet to table
```

Use `cabinet` as the source for each object, but still grasp and place one object at a time.

## Pose Query Tasks

For tasks asking where the robot is, use `robot_pose` first, then finish after the pose observation succeeds.
