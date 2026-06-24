# Decision Maker Planned Architecture

This document describes the planned high-level architecture for improving the
current `decision_maker` package. The goal is to keep the Jetson Thor runtime
lightweight while improving complex natural-language task planning, object
search, visual grounding, and failure recovery.

## Motivation

The current robot uses a local Qwen 8B model on Jetson Thor to reason over
tasks and choose capabilities such as `object_query`, `navigation`, and
`grasp_place`. This is suitable for simple commands, but complex or human-like
instructions can exceed the local model's planning ability.

The future system should also use a 3D scene graph derived from the map.
However, the current 3D map mainly contains large static objects such as
tables, chairs, cabinets, and sofas. It does not reliably contain small movable
objects such as Pringles, bottles, cups, or snacks. Because small objects are
dynamic, they should be discovered and updated during task execution instead of
being assumed to exist in the initial map.

The planned architecture therefore combines:

- a local Qwen 8B closed-loop planner on Jetson Thor
- a server-side large model for semantic calibration
- an avatar interface for minimal user clarification
- a VLM view critic before grasp execution
- an anchor-guided dynamic belief graph
- a lightweight execution gate for state and perception checks

## Core Idea

The local Qwen 8B model remains the main runtime planner. It does not need to
solve the entire task in one long plan. Instead, it repeatedly chooses the next
simple robot capability based on the current task state and recent observations.

The server model and avatar are not the main robot controllers. They are used
only when the local system has semantic uncertainty or user intent ambiguity.

The belief graph stores what the robot currently believes about the environment.
It starts with large static objects from the 3D map and gradually adds small
dynamic objects observed during tasks.

## Component Roles

### Jetson Qwen 8B: Local Closed-Loop Planner

The local Qwen 8B model runs on Jetson Thor and is responsible for choosing the
next robot capability.

Example capability calls:

```json
{"capability": "object_query", "target": "pringles"}
```

```json
{"capability": "navigation", "target": "cabinet"}
```

```json
{"capability": "grasp_place", "action": "grasp", "target": "pringles"}
```

Main responsibilities:

- parse simple task frames
- choose the next capability
- use recent observations for replanning
- keep execution local and low-latency
- avoid long-horizon reasoning when possible

### Server Large Model: Semantic Calibrator

The server-side large model is used only when the local model is uncertain or
the instruction is semantically complex.

Example triggers:

- ambiguous human instruction
- complex multi-object task
- implicit object or destination
- low local parsing confidence
- unclear relationship between task and scene graph

The server model should return a corrected task frame or clarification proposal,
not directly control the robot.

Example output:

```json
{
  "intent": "bring",
  "object": "pringles",
  "source": "cabinet",
  "destination": "table",
  "need_user_clarification": false
}
```

### Avatar: Minimal Clarification Interface

The avatar is used only when the system cannot safely infer the user's intent.
It should ask a small, concrete question instead of starting an open-ended chat.

Example:

```text
你是指櫃子上的品客，還是桌上的餅乾？
```

The avatar is triggered when:

- the task has multiple plausible meanings
- the wrong action may cause an undesirable result
- the server model cannot confidently resolve the ambiguity

### VLM View Critic: Mandatory Pre-Grasp Visual Gate

Before every `grasp_place` action with `action = grasp`, the system must call a
VLM view critic to check whether the target object is currently visible and
whether the view is sufficient for grasping.

The VLM view critic answers:

- Is the target visible?
- Is the view sufficient for grasp?
- If not, how should the robot adjust its view?

Example output:

```json
{
  "sufficient_view": false,
  "target_visible": "unknown",
  "recommended_motion": {
    "type": "rotate",
    "direction": "right",
    "yaw_deg": 25
  }
}
```

Important design choice:

- `object_query` estimates where the object or source should be.
- `VLM view critic` checks whether the object is actually visible now.
- `view_adjustment` changes the robot/camera viewpoint.
- `grasp_place` executes only after the visual gate passes.

After view adjustment, the system should run the VLM view critic again. It does
not need to repeat `object_query` unless the object location itself is still
unknown or invalid.

### Lightweight Execution Gate

The system does not require a heavy capability contract for every action.
Since the planner only emits predefined capabilities, a lightweight execution
gate is enough.

The execution gate checks deterministic state and perception rules:

- `place` requires the robot to be holding the target object
- `grasp` requires the robot to be near the target or source anchor
- every `grasp` requires VLM view critic approval
- unknown small objects should be searched near known source anchors when
  available
- ambiguous tasks should be routed to the server model or avatar

This gate is not a second LLM planner. It is a small rule-based safety and
perception layer.

## Belief Graph Design

The belief graph has two layers.

### Static Anchor Graph

The static anchor graph is initialized from the 3D semantic map. It contains
large stable objects and spatial landmarks.

Examples:

- table
- chair
- cabinet
- sofa
- counter
- shelf

These objects are used as anchors for navigation and object search.

Example:

```json
{
  "cabinet_1": {
    "type": "anchor",
    "class": "cabinet",
    "pose": {"x": 2.1, "y": 0.8},
    "stable": true
  },
  "table_1": {
    "type": "anchor",
    "class": "table",
    "pose": {"x": 0.4, "y": 1.6},
    "stable": true
  }
}
```

### Dynamic Object Belief Graph

Small movable objects are added during task execution through object query,
VLM observation, grasp feedback, place feedback, or user clarification.

Examples:

- Pringles
- bottle
- cup
- apple
- remote

Example dynamic object node:

```json
{
  "pringles_1": {
    "type": "dynamic_object",
    "class": "pringles",
    "pose": {"x": 2.2, "y": 0.9},
    "on": "cabinet_1",
    "confidence": 0.78,
    "visibility": 0.72,
    "held_by": null,
    "last_seen": "now"
  }
}
```

This design allows the robot to start with only large map objects and learn
small object locations over time.

## Overall Workflow

```text
User instruction
   ↓
Task understanding
   ├── local Qwen 8B parse
   ├── server LLM calibration if needed
   └── avatar clarification if still ambiguous
   ↓
Task frame
   ↓
Belief graph retrieval
   ├── static anchor graph
   └── dynamic object belief graph
   ↓
Qwen 8B closed-loop planner
   ↓
Simple capability call
   ↓
Lightweight execution gate
   ├── state rule checks
   ├── semantic uncertainty routing
   └── mandatory VLM gate before grasp
   ↓
ROS capability execution
   ├── object_query
   ├── navigation
   ├── view_critic
   ├── view_adjustment
   ├── grasp_place
   └── finish
   ↓
Observation update
   ↓
Belief graph update
   ↓
Repeat until finish
```

## Example Task Flow

Task:

```text
把櫃子上的品客拿到桌上
```

Expected flow:

```text
1. Parse task:
   object = pringles
   source = cabinet
   destination = table

2. Query belief graph:
   cabinet is known from static anchor graph
   table is known from static anchor graph
   pringles is not yet known

3. Qwen 8B chooses:
   object_query(pringles)

4. Object query fails:
   pringles is not found in current dynamic object belief

5. Qwen 8B chooses:
   object_query(cabinet)

6. Object query succeeds:
   cabinet pose is available

7. Qwen 8B chooses:
   navigation(cabinet)

8. Robot navigates to cabinet.

9. Qwen 8B chooses:
   grasp_place(grasp, pringles)

10. Execution gate intercepts grasp:
    run VLM view critic first

11. If VLM says view is insufficient:
    view_adjustment(rotate_right_25_deg)
    run VLM view critic again

12. If VLM says target is visible and view is sufficient:
    update belief graph with pringles_1 near/on cabinet_1
    execute grasp_place(grasp, pringles)

13. Update belief graph:
    pringles_1 held_by robot

14. Qwen 8B chooses:
    navigation(table)

15. Qwen 8B chooses:
    grasp_place(place, pringles, destination=table)

16. Update belief graph:
    pringles_1 on table_1

17. finish
```

## Planned Implementation Modules

### `BeliefGraphManager`

Responsible for:

- loading static anchor objects from the 3D map or scene graph
- storing dynamic object nodes
- retrieving task-relevant graph slices
- updating object confidence, visibility, and relations
- recording held object and placed object states

### `ServerLLMCalibrator`

Responsible for:

- complex instruction interpretation
- task frame correction
- ambiguity detection
- generating minimal clarification candidates

### `AvatarClarifier`

Responsible for:

- asking user-facing clarification questions
- converting user answers back into task frame updates

### `VLMViewCritic`

Responsible for:

- checking target visibility before grasp
- deciding if the current view is sufficient
- recommending view adjustment commands

### `ExecutionGate`

Responsible for:

- deterministic state checks
- mandatory pre-grasp VLM routing
- preventing invalid place/grasp sequences
- routing semantic uncertainty to server/avatar

## MVP Milestones

### Milestone 1: Belief Graph Skeleton

- Create static anchor graph from known large objects.
- Add dynamic object storage.
- Update dynamic objects after successful object query, grasp, and place.

### Milestone 2: Pre-Grasp VLM Gate

- Add `view_critic` capability.
- Add `view_adjustment` capability.
- Force every grasp to pass VLM view critic before execution.

### Milestone 3: Lightweight Execution Gate

- Add deterministic checks before execution.
- Intercept invalid or visually unsafe actions.
- Keep Qwen 8B output simple.

### Milestone 4: Server Semantic Calibration

- Add server-side large model endpoint.
- Use it only for ambiguous or complex task understanding.
- Return task frames, not direct robot commands.

### Milestone 5: Avatar Clarification

- Ask minimal clarification questions only when needed.
- Feed user clarification back into the task frame and belief graph.

## Research Novelty

This planned architecture can be framed as:

```text
Anchor-guided dynamic belief graph with local closed-loop LLM planning,
mandatory pre-grasp VLM visual gating, and multi-level semantic calibration.
```

Main novelty points:

- The initial scene graph does not need to contain all small objects.
- Large static objects act as anchors for searching dynamic objects.
- Small objects are incrementally added through task execution.
- The Jetson model remains the local executor-planner.
- The server model is used as a semantic calibrator, not a remote controller.
- The avatar is used only for minimal ambiguity resolution.
- Every grasp is guarded by visual sufficiency checking.
