package com.marswars.util;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public abstract class NumUtil {

    public static boolean isNear(Rotation2d x, Rotation2d y, double epsilon) {
        return Math.abs(x.minus(y).getRadians()) < epsilon;
    }

    public static boolean isNear(Translation2d x, Translation2d y, double epsilon) {
        return x.getDistance(y) < epsilon;
    }

    public static boolean isNear(
            Pose2d Pose2dA, Pose2d Pose2dB, double epsilon_rotation, double epsilon_translation) {
        return (isNear(Pose2dA.getRotation(), Pose2dB.getRotation(), epsilon_rotation)
                && isNear(
                        Pose2dA.getTranslation(), Pose2dB.getTranslation(), epsilon_translation));
    }

    public static boolean isNear(ChassisVelocities ch1, ChassisVelocities ch2, double epsilon) {
        return MathUtil.isNear(ch1.vx, ch2.vx, epsilon)
                && MathUtil.isNear(ch1.vy, ch2.vy, epsilon);
    }

    public static Transform2d flatten(Transform3d tf) {
        return new Transform2d(tf.getX(), tf.getY(), tf.getRotation().toRotation2d());
    }
}
