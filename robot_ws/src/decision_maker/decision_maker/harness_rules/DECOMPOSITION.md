# Task Decomposition Rules

This document is used only before the closed-loop agent planner runs. Its purpose is to split a user command into ordered atomic subtasks. It does not define capability-call JSON; that belongs to OUTPUT_FORMAT.md.

## Goal

Convert one user command into a small list of subtasks that can each be handled by the existing agent-based planner.

## Atomic Subtasks

A subtask should describe one robot-level objective, such as:

- `bring bottle to table`
- `move bottle from sofa to table`
- `bring apple to cabinet`
- `if bottle is on sofa, bring bottle to table`

Do not output low-level capability calls such as `object_query`, `navigation`, or `grasp_place` in this stage. Those are chosen later by the agent planner.

## Multi-Object Commands

For objects joined by `and` or commas, create one subtask per object. Preserve the natural order from the command.

Example:

```text
bring bottle and apple to table
```

Expected subtasks:

1. `bring bottle to table`
2. `bring apple to table`

## Sequential Commands

For `then`, `and then`, or `after that`, preserve the order exactly.

Example:

```text
move bottle from sofa to table, then bring apple to cabinet
```

Expected subtasks:

1. `move bottle from sofa to table`
2. `bring apple to cabinet`

## Move Commands

For commands like `move <object> from <source> to <destination>`, keep the source and destination in the subtask fields.

## Place Versus Handover

- `bring`, `move`, `take`, or `carry` an object to a place, surface, or piece of furniture is a place task. Keep it as `type=bring` or `type=move`.
- Use `type=handover` only when the user explicitly says `handover`, `hand over`, `give`, or `pass`, or explicitly identifies a human receiver.
- Never infer handover merely because an object is transported to a destination.

## Direct Grasp And Place

For `grasp <object> and place it on <destination>` with no source, use `type=direct_grasp_place` and preserve the grasp-first wording. The robot is already in front of the object, so the planner must not query or navigate to the object before grasping.

After grasp succeeds, query and navigate to the destination, then place the object.

For `grasp <object> on/from <source> and place it on <destination>`, preserve the source and use a normal `type=bring` transfer. Query/navigate the object or its source before grasping, then query/navigate the destination and place the object.

## Conditional Commands

For simple conditional commands, keep the condition in the subtask text and resolve simple pronouns when possible.

Example:

```text
if the bottle is on sofa, bring it to table
```

Expected subtask:

```text
if bottle is on sofa, bring bottle to table
```
