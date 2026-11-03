package frc.robot.constants;

import coppercore.wpilib_interface.alliance_util.AllianceUtil;
import coppercore.wpilib_interface.alliance_util.AllianceUtil.AllianceBasedValue;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.driverstation.Alliance;

/**
 * The AllianceBasedFieldConstants class provides methods for getting relevant field locations from
 * FieldConstants.java based on which alliance we're on.
 */
public class AllianceBasedFieldConstants {
  public static final AllianceBasedValue<Translation3d> hubInnerCenterPoint =
      new AllianceBasedValue<>(
          () ->
              AllianceUtil.isRed()
                  ? FieldConstants.Hub.oppInnerCenterPoint()
                  : FieldConstants.Hub.innerCenterPoint());

  public static final AllianceBasedValue<Translation2d> hubCenterPoint2d =
      new AllianceBasedValue<>(
          () ->
              AllianceUtil.isRed()
                  ? FieldConstants.Hub.oppInnerCenterPoint().toTranslation2d()
                  : FieldConstants.Hub.innerCenterPoint().toTranslation2d());

  /**
   * Check whether the given pose is within the alliance zone.
   *
   * @param robotPose A Pose2d containing the current position of the robot to verify
   * @return {@code true} if the robot is within its own alliance zone, {@code false} otherwise.
   */
  public static final boolean isInAllianceZone(Pose2d robotPose) {
    Alliance alliance = AllianceUtil.getAlliance();

    return switch (alliance) {
      case RED -> robotPose.getX() > FieldConstants.LinesVertical.oppAllianceZone();
      case BLUE -> robotPose.getX() < FieldConstants.LinesVertical.allianceZone();
    };
  }

  /**
   * Check whether the given pose is within the opponent alliance zone.
   *
   * @param robotPose A Pose2d containing the current position of the robot to verify
   * @return {@code true} if the robot is within the opponent alliance zone, {@code false}
   *     otherwise.
   */
  public static final boolean isInOppAllianceZone(Pose2d robotPose) {
    Alliance alliance = AllianceUtil.getAlliance();

    return switch (alliance) {
      case RED -> robotPose.getX() < FieldConstants.LinesVertical.allianceZone();
      case BLUE -> robotPose.getX() > FieldConstants.LinesVertical.oppAllianceZone();
    };
  }
}
