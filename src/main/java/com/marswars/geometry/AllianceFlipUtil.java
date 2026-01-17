package com.marswars.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Utility class for flipping geometric coordinates between alliance perspectives in FRC.
 * This class provides methods to transform coordinates, poses, and regions based on field symmetry.
 * 
 * <p>The utility supports two types of field symmetry:
 * <ul>
 *   <li>DIAGONAL - Rotates coordinates 180 degrees around the field center</li>
 *   <li>DIRECT - Mirrors coordinates horizontally across the field center</li>
 * </ul>
 * 
 * <p>Before using any transformation methods, the field geometry must be configured using
 * {@link #configureFieldGeometry(SymmetryType, Translation2d)}.
 * 
 */
public class AllianceFlipUtil {

  /**
   * Enumeration of field symmetry types for alliance coordinate transformation.
   */
  public enum SymmetryType {
    /** Diagonal symmetry - rotates coordinates 180 degrees around field center */
    DIAGONAL,
    /** Direct symmetry - mirrors coordinates horizontally across field center */
    DIRECT
  }

  /** The configured symmetry type for field transformations */
  static SymmetryType symmetry_type_ = null;
  /** The center point of the field for transformation calculations */
  static Translation2d field_center_ = null;

  /**
   * Configures the field geometry for alliance coordinate transformations.
   * This method must be called before using any transformation methods.
   * 
   * @param symmetry the type of field symmetry (DIAGONAL or DIRECT)
   * @param field_center the center point of the field
   */
  public static void configureFieldGeometry(SymmetryType symmetry, Translation2d field_center) {
    symmetry_type_ = symmetry;
    field_center_ = field_center;
  }

  /**
   * Applies alliance flip transformation to a 2D translation.
   * 
   * @param translation the original translation to transform
   * @return the transformed translation from the opposite alliance perspective
   * @throws IllegalStateException if the utility has not been configured
   */
  public static Translation2d apply(Translation2d translation) {

    // Ensure the utility has been configured
    if( symmetry_type_ == null || field_center_ == null) {
      throw new IllegalStateException("AllianceFlipUtil not configured");
    }

    if (symmetry_type_ == SymmetryType.DIAGONAL) {
      return translation.rotateAround(field_center_, Rotation2d.fromDegrees(180));
    } else {
      double distToMid = Math.abs(translation.getX() - field_center_.getX());
      double offset = 0;
      if (translation.getX() < field_center_.getX()) {
        offset = distToMid * 2;
      } else {
        offset = -(distToMid * 2);
      }
      return new Translation2d(translation.getX() + offset, translation.getY());
    }
  }

  /**
   * Applies alliance flip transformation to a 3D translation.
   * 
   * @param translation the original 3D translation to transform
   * @return the transformed 3D translation from the opposite alliance perspective
   * @throws IllegalStateException if the utility has not been configured
   */
  public static Translation3d apply(Translation3d translation) {

    // Ensure the utility has been configured
    if( symmetry_type_ == null || field_center_ == null) {
      throw new IllegalStateException("AllianceFlipUtil not configured");
    }

    if (symmetry_type_ == SymmetryType.DIAGONAL) {
      return translation.rotateAround(
          new Translation3d(field_center_),
          new Rotation3d(Rotation2d.fromDegrees(180)));
    } else {
      double distToMid = Math.abs(translation.getX() - field_center_.getX());
      double offset = 0;
      if (translation.getX() < field_center_.getX()) {
        offset = distToMid * 2;
      } else {
        offset = -(distToMid * 2);
      }
      return new Translation3d(translation.getX() + offset, translation.getY(), translation.getZ());
    }
  }

  /**
   * Applies alliance flip transformation to a 2D pose.
   * Transforms both the translation and rotation components.
   * 
   * @param pose the original 2D pose to transform
   * @return the transformed 2D pose from the opposite alliance perspective
   * @throws IllegalStateException if the utility has not been configured
   */
  public static Pose2d apply(Pose2d pose) {
    return new Pose2d(
        apply(pose.getTranslation()),
        pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
  }

  /**
   * Applies alliance flip transformation to a 3D pose.
   * Transforms both the translation and rotation components.
   * 
   * @param pose the original 3D pose to transform
   * @return the transformed 3D pose from the opposite alliance perspective
   * @throws IllegalStateException if the utility has not been configured
   */
  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(
        apply(pose.getTranslation()),
        pose.getRotation().rotateBy(new Rotation3d(Rotation2d.fromDegrees(180))));
  }

  /**
   * Applies alliance flip transformation to a region.
   * Automatically determines the region type and applies the appropriate transformation.
   * 
   * @param region the original region to transform (PolygonRegion or CircularRegion)
   * @return the transformed region from the opposite alliance perspective
   * @throws IllegalStateException if the utility has not been configured
   */
  public static Region apply(Region region) {
    if (region instanceof PolygonRegion) {
      return apply((PolygonRegion) region);
    } else {
      return apply((CircularRegion) region);
    }
  }

  /**
   * Applies alliance flip transformation to a polygon region.
   * Transforms all vertices of the polygon.
   * 
   * @param region the original polygon region to transform
   * @return the transformed polygon region from the opposite alliance perspective
   * @throws IllegalStateException if the utility has not been configured
   */
  public static PolygonRegion apply(PolygonRegion region) {
    Translation2d[] points = region.getPoints();
    for (int i = 0; i < points.length; i++) {
      points[i] = apply(points[i]);
    }
    return new PolygonRegion(points, region.getName());
  }

  /**
   * Applies alliance flip transformation to a circular region.
   * Transforms the center point while preserving the radius.
   * 
   * @param region the original circular region to transform
   * @return the transformed circular region from the opposite alliance perspective
   * @throws IllegalStateException if the utility has not been configured
   */
  public static CircularRegion apply(CircularRegion region) {
    Translation2d newCenter;
    newCenter = apply(region.getCenter());
    return new CircularRegion(newCenter, region.getRadius(), region.getName());
  }

  /**
   * Applies alliance flip transformation to a tight rope path.
   * Transforms both endpoint poses and swaps their order to maintain path direction.
   * 
   * @param tightRope the original tight rope path to transform
   * @return the transformed tight rope path from the opposite alliance perspective
   * @throws IllegalStateException if the utility has not been configured
   */
  public static TightRope apply(TightRope tightRope) {
    return new TightRope(
        apply(tightRope.pose_b_),
        apply(tightRope.pose_a_),
        tightRope.getName());
  }
}
