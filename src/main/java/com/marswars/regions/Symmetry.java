package com.marswars.regions;

/** How one alliance's half of the field maps onto the other's. */
public enum Symmetry {
    /** Turned 180 degrees about the field center: both x and y flip. 2022, 2026. */
    ROTATIONAL,
    /** Mirrored across the center line: only x flips. 2023, 2024, 2025. */
    MIRRORED
}
