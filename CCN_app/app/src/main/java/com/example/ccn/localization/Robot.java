package com.example.ccn.localization;

import com.aldebaran.qi.Future;
import androidx.annotation.NonNull;

/**
 * A robotic component
 */
public interface Robot {
    /**
     * Mean to be called when the robotic component must stop.
     * This is the place where related actions must be stopped.
     *
     * @return A {link Future} that is a success when the robotic
     * component has correctly stopped.
     */
    @NonNull
    Future<Void> stop();
}
