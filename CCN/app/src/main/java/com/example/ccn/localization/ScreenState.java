package com.example.ccn.localization;

/**
 * A state for screens. Indicates the current screen.
 */
public enum ScreenState {
    NONE,
    LOCALIZATION_MENU,
    LOCALIZE,
    GO_TO_GOAL,
    GO_TO_GOAL_BASELINE,
    GO_TO_GOAL_FOREKNOWLEDGE,
    GO_TO_GOAL_ADAPTATION,
    END,
}