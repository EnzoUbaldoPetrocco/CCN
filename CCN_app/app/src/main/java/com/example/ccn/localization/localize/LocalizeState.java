/*
 * Copyright (C) 2018 SoftBank Robotics Europe
 * See COPYING for the license
 */
package com.example.ccn.localization.localize;

/**
 * A state for {@link LocalizeScreen}.
 */
enum LocalizeState {
    IDLE,
    BRIEFING,
    ADVICES,
    LOADING_MAP,
    LOCALIZING,
    ERROR,
    SUCCESS,
    END,
}