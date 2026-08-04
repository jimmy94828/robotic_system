# Verification Rules

The harness checks high-level consistency before or after capability execution.

## Invalid or Suspicious Calls

- Unknown capability is invalid.
- `object_query` without `target` is invalid.
- `navigation` without `target` or `pose` is invalid.
- `grasp_place` without valid `action` is invalid.
- `place` without `destination` is invalid.
- `handover` without `target` is invalid.
- `finish` without `task_done=true` is invalid.
- `object_query` targeting a person/human/receiver is invalid; query the referenced location.
- Reject `action=handover` unless the task explicitly requests handover/give/pass or identifies a human receiver.
- Reject `action=place` when the task explicitly requests a handover or identifies a human receiver.

## Order Constraints

- Direct grasp-only tasks may call `grasp_place` immediately without prior `object_query` or `navigation`.
- For transfer tasks, querying and navigating to the source should happen before grasp.
- Placing or handing over should happen after a successful grasp.
- Destination lookup/navigation should happen before placing or handing over.
- Do not finish immediately after a failed capability.
- Do not repeat the exact same failed call unless new information has been observed.

## Target Constraints

- Do not use combined relational phrases as action targets, such as `pringles on table`.
- Use the source receptacle/place for source lookup, such as `table`.
- Use the object itself for grasp/place/handover target, such as `pringles`.
