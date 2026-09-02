package com.marswars.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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

    public static boolean isNear(ChassisSpeeds ch1, ChassisSpeeds ch2, double epsilon) {
        return MathUtil.isNear(ch1.vxMetersPerSecond, ch2.vxMetersPerSecond, epsilon)
                && MathUtil.isNear(ch1.vyMetersPerSecond, ch2.vyMetersPerSecond, epsilon);
    }

    public static Transform2d flatten(Transform3d tf) {
        return new Transform2d(tf.getX(), tf.getY(), tf.getRotation().toRotation2d());
    }

    // TODO(human): implement packBits and unpackBits.
    //
    // These convert between a boolean[] (e.g. "which of these 12 reef branches are occupied") and
    // a plain int (the only bitfield-shaped type NetworkTables can carry on the wire). They're used
    // by DashboardBridge consumers to interpret the raw ints that come out of a DashboardChannel.
    //
    // packBits: for each index i where bits[i] is true, set bit i of the result.
    //   e.g. {true, false, true} -> bit 0 set, bit 2 set -> 0b101 -> 5
    //
    // unpackBits: the inverse -- for each of the first `count` bits of `packed`, produce a boolean.
    //   e.g. unpackBits(5, 3) -> {true, false, true}
    //
    // Hint: `1 << i` produces a value with only bit i set; `|=` to set a bit, `&` plus `!= 0` to
    // test one.

    public static int packBits(boolean[] bits) {
        int packed = 0;
        for (int i = 0; i < bits.length; i++) {
            if (bits[i]) {
                packed |= 1 << i;
            }
        }
        return packed;
    }

    public static boolean[] unpackBits(int packed, int count) {
        boolean[] bits = new boolean[count];
        for (int i = 0; i < count; i++) {
            bits[i] = (packed & (1 << i)) != 0;
        }
        return bits;
    }
}
