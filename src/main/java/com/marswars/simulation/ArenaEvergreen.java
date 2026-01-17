package com.marswars.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

/**
 *
 *
 * <h1>The playing field for the 2025 FRC Game: Reefscape</h1>
 *
 * <p>This class represents the playing field for the 2025 FRC game, Reefscape.
 *
 * <p>It extends {@link SimulatedArena} and includes specific details of the Reefscape game
 * environment.
 */
public class ArenaEvergreen extends SimulatedArena {
    public static final class EvergreenObstacleMap extends FieldMap {

        private static final double FIELD_LENGTH_METERS =
                Units.feetToMeters(54) + Units.inchesToMeters(1);
        private static final double FIELD_WIDTH_METERS =
                Units.feetToMeters(26) + Units.inchesToMeters(7);

        /**
         * Constructs the obstacle map for the Evergreen field.
         * Adds border lines for all four walls of the field.
         */
        public EvergreenObstacleMap() {
            super();

            // blue wall
            super.addBorderLine(
                    new Translation2d(0.0, 0.0), new Translation2d(0.0, FIELD_WIDTH_METERS));

            // red wall
            super.addBorderLine(
                    new Translation2d(FIELD_LENGTH_METERS, 0.0),
                    new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS));

            // left walls
            super.addBorderLine(
                    new Translation2d(0.0, FIELD_WIDTH_METERS),
                    new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS));

            // right walls
            super.addBorderLine(
                    new Translation2d(0.0, 0.0), new Translation2d(FIELD_LENGTH_METERS, 0.0));
        }
    }

    /**
     * Constructs a new ArenaEvergreen simulation arena.
     * Initializes the arena with the Evergreen field obstacle map.
     */
    public ArenaEvergreen() {
        super(new EvergreenObstacleMap());
    }

    /**
     * Places game pieces on the field at their starting positions.
     * For Evergreen, no game pieces are pre-placed on the field.
     */
    @Override
    public void placeGamePiecesOnField() {
        // No pre-placed game pieces in Evergreen
    }

    /**
     * Gets the poses of all game pieces of the specified type.
     * 
     * @param type the type of game pieces to retrieve
     * @return a list of poses for game pieces of the specified type
     */
    @Override
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        // No game pieces on field elements in Evergreen
        List<Pose3d> poses = super.getGamePiecesPosesByType(type);
        return poses;
    }

    /**
     * Clears all game pieces from the field.
     * This method removes all game pieces that may have been placed on the field.
     */
    @Override
    public synchronized void clearGamePieces() {
        super.clearGamePieces();
    }
}
