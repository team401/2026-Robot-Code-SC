package frc.robot.constants;

import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Distance;

/** ManualModeConstants contains constants that define how to shoot when vision isn't used */
public class ManualModeConstants {
  public final Distance assumedPassDistance = Meters.of(4.0);

  public final Rotation2d bluePassHeading = Rotation2d.kZero;
  public final Rotation2d redPassHeading = Rotation2d.k180deg;

  public final Distance assumedHubDistance = Meters.of(3.1);

  public final Rotation2d blueHubHeading = Rotation2d.k180deg;
  public final Rotation2d redHubHeading = Rotation2d.kZero;
}
