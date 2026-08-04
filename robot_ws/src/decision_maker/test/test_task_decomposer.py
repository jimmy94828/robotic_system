from decision_maker.task_decomposer import TaskDecomposer, validate_decomposition


def _texts(task):
    return TaskDecomposer().decompose_texts(task)


def test_decompose_multi_object_bring():
    assert _texts("bring bottle and apple to table") == [
        "bring bottle to table",
        "bring apple to table",
    ]


def test_decompose_move_then_bring():
    assert _texts("move bottle from sofa to table, then bring apple to cabinet") == [
        "move bottle from sofa to table",
        "bring apple to cabinet",
    ]


def test_decompose_bring_then_move():
    assert _texts("bring apple to table, then move bottle from chair to cabinet") == [
        "bring apple to table",
        "move bottle from chair to cabinet",
    ]


def test_decompose_simple_conditional_with_pronoun_resolution():
    plan = TaskDecomposer().decompose("if the bottle is on sofa, bring it to table")

    assert [subtask["text"] for subtask in plan["subtasks"]] == [
        "if bottle is on sofa, bring bottle to table"
    ]
    assert plan["subtasks"][0] == {
        "subtask_id": 1,
        "text": "if bottle is on sofa, bring bottle to table",
        "type": "conditional",
        "object": "bottle",
        "source": "sofa",
        "destination": "table",
    }


def test_validate_decomposition_normalizes_ids_and_fields():
    plan = validate_decomposition(
        "bring apple to table",
        {
            "original_task": "bring apple to table",
            "subtasks": [
                {
                    "subtask_id": 99,
                    "text": " Bring Apple to Table ",
                    "type": "BRING",
                    "object": "Apple",
                    "source": None,
                    "destination": "Table",
                }
            ],
        },
    )

    assert plan == {
        "original_task": "bring apple to table",
        "subtasks": [
            {
                "subtask_id": 1,
                "text": "bring apple to table",
                "type": "bring",
                "object": "apple",
                "source": None,
                "destination": "table",
            }
        ],
    }


def test_validate_decomposition_corrects_handover_bias_for_place_destination():
    plan = validate_decomposition(
        "bring pringles on table to chair",
        {
            "original_task": "bring pringles on table to chair",
            "subtasks": [
                {
                    "subtask_id": 1,
                    "text": "handover pringles on table to chair",
                    "type": "handover",
                    "object": "pringles",
                    "source": "table",
                    "destination": "chair",
                }
            ],
        },
    )

    assert plan["subtasks"][0] == {
        "subtask_id": 1,
        "text": "bring pringles on table to chair",
        "type": "bring",
        "object": "pringles",
        "source": "table",
        "destination": "chair",
    }


def test_explicit_handover_is_preserved():
    plan = validate_decomposition(
        "hand it over to me at the chair",
        {
            "subtasks": [
                {
                    "text": "handover pringles to chair",
                    "type": "handover",
                    "object": "pringles",
                    "source": None,
                    "destination": "chair",
                }
            ]
        },
    )

    assert plan["subtasks"][0]["type"] == "handover"
    assert plan["subtasks"][0]["text"] == "handover pringles to chair"


def test_bring_to_explicit_human_receiver_becomes_handover():
    plan = TaskDecomposer().decompose("bring pringles on table to me at chair")

    assert plan["subtasks"][0] == {
        "subtask_id": 1,
        "text": "handover pringles on table to chair",
        "type": "handover",
        "object": "pringles",
        "source": "table",
        "destination": "chair",
    }


def test_direct_grasp_place_overrides_model_source_navigation_plan():
    plan = validate_decomposition(
        "grasp the pringle and place it on the chair.",
        {
            "subtasks": [
                {
                    "subtask_id": 1,
                    "text": "bring pringle on table to chair",
                    "type": "bring",
                    "object": "pringle",
                    "source": "table",
                    "destination": "chair",
                }
            ]
        },
    )

    assert plan["subtasks"] == [
        {
            "subtask_id": 1,
            "text": "grasp pringle and place it on chair",
            "type": "direct_grasp_place",
            "object": "pringle",
            "source": None,
            "destination": "chair",
        }
    ]


def test_grasp_place_with_explicit_source_preserves_source_navigation():
    plan = validate_decomposition(
        "grasp the pringles on the table and place it on the chair",
        {
            "subtasks": [
                {
                    "text": "grasp pringles and place it on chair",
                    "type": "direct_grasp_place",
                    "object": "pringles",
                    "source": None,
                    "destination": "chair",
                }
            ]
        },
    )

    assert plan["subtasks"] == [
        {
            "subtask_id": 1,
            "text": "bring pringles on table to chair",
            "type": "bring",
            "object": "pringles",
            "source": "table",
            "destination": "chair",
        }
    ]


def test_verbose_direct_grasp_place_is_preserved():
    plan = validate_decomposition(
        "It is a grasp and place task, please grasp the pringle and place it on the chair.",
        {
            "subtasks": [
                {
                    "text": "bring pringle on table to chair",
                    "type": "bring",
                    "object": "pringle",
                    "source": "table",
                    "destination": "chair",
                }
            ]
        },
    )

    assert plan["subtasks"][0]["type"] == "direct_grasp_place"
    assert plan["subtasks"][0]["text"] == "grasp pringle and place it on chair"


def test_hand_it_to_person_on_chair_becomes_handover_to_chair():
    task = "Grasp the can from the desk and hand it to the person sitting on the chair"
    plan = validate_decomposition(
        task,
        {
            "subtasks": [
                {
                    "text": "bring can on desk to person on chair",
                    "type": "bring",
                    "object": "can",
                    "source": "desk",
                    "destination": "person on chair",
                }
            ]
        },
    )

    assert plan["subtasks"] == [
        {
            "subtask_id": 1,
            "text": "handover can on desk to chair",
            "type": "handover",
            "object": "can",
            "source": "desk",
            "destination": "chair",
        }
    ]
