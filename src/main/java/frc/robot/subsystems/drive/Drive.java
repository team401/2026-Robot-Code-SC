// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import com.pathplanner.lib.util.PathPlannerLogging;
import coppercore.controls.ServiceThread;
import coppercore.monitors.TotalCurrentCalculator;
import coppercore.wpilib_interface.DriveTemplate;
import coppercore.wpilib_interface.alliance_util.AllianceUtil;
import coppercore.wpilib_interface.tuning.PIDGains;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.units.measure.Current;
import org.wpilib.driverstation.Alert;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.constants.JsonConstants;
import frc.robot.util.littletonUtil.PoseEstimator;
import frc.robot.util.littletonUtil.PoseEstimator.TimestampedVisionUpdate;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements DriveTemplate {

  // PathPlanner config constants
  // private final double ROBOT_MASS_KG = JsonConstants.robotInfo.robotMass.in(Kilograms);
  // private final double ROBOT_MOI = JsonConstants.robotInfo.robotMOI.in(KilogramSquareMeters);
  // private final double WHEEL_COF = JsonConstants.robotInfo.wheelCof;
  // // private final RobotConfig PP_CONFIG =
  //     new RobotConfig(
  //         ROBOT_MASS_KG,
  //         ROBOT_MOI,
  //         new ModuleConfig(
  //             JsonConstants.physicalDriveConstants.FrontLeft.WheelRadius,
  //             JsonConstants.physicalDriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
  //             WHEEL_COF,
  //             DCMotor.getKrakenX60Foc(1)
  //                 .withReduction(
  //                     JsonConstants.physicalDriveConstants.FrontLeft.DriveMotorGearRatio),
  //             JsonConstants.physicalDriveConstants.FrontLeft.SlipCurrent,
  //             1),
  //         getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", Alert.Level.HIGH);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private Rotation2d lastGyroYaw = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private PoseEstimator maPoseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

  // Log a field2d for Elastic
  private final Field2d field2d = new Field2d();

  // Track if we're in defense mode to know when to lock the wheels
  private boolean inDefenseMode = false;

  // Track if the X-lock button is pressed to know when to lock the wheels
  private boolean xLockPressed = false;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, JsonConstants.physicalDriveConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, JsonConstants.physicalDriveConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, JsonConstants.physicalDriveConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, JsonConstants.physicalDriveConstants.BackRight);

    // Ensure that ServiceThread class is loaded and defaultServiceThread is running
    ServiceThread.defaultServiceThread.queueCommand(
        () -> System.out.println("Default Service Thread has started"));

    Command resetToAutoLimitsCommand =
        new InstantCommand(
                () -> {
                  ServiceThread.defaultServiceThread.queueCommand(
                      () ->
                          setSupplyCurrentLimit(
                              JsonConstants.physicalDriveConstants.driveSupplyCurrentLimit));
                })
            .ignoringDisable(true);

    SmartDashboard.putData("ResetToAutoCurrentLimits", resetToAutoLimitsCommand);

    // Usage reporting for swerve template
    HAL.reportUsage("RobotDrive", "Swerve_AdvantageKit");

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    // AutoBuilder.configure(
    //     this::getPose,
    //     this::setPose,
    //     this::getChassisSpeeds,
    //     this::runVelocity,
    //     new PPHolonomicDriveController(
    //         new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
    //     PP_CONFIG,
    //     () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    //     this);
    // Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
          field2d.getObject("traj").setPoses(activePath);
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Since Drive is placed within an Optional, it can't be found in a recursive down from Robot
    AutoLogOutputManager.addObject(this);

    SmartDashboard.putData("Field2d", field2d);
  }

  @Override
  public void periodic() {
    long startTimeUs = RobotController.getTime();
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    double supplyCurrentSum = 0.0;
    for (var module : modules) {
      module.periodic();
      supplyCurrentSum += module.getSupplyCurrentAmps();
    }
    odometryLock.unlock();

    TotalCurrentCalculator.recordCurrent(hashCode(), supplyCurrentSum);

    // Stop moving when disabled
    if (DriverStationBackend.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStationBackend.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    Pose2d totalDeltaPose = Pose2d.kZero;

    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance
                    - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // This twist is used later in multiple branches, so we just create it here preemptively
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      if (JsonConstants.featureFlags.useMAPoseEstimator) {
        // The following code was added for the support of ma pose estimator
        Rotation2d deltaYaw;
        if (i == 0) {
          deltaYaw = rawGyroRotation.minus(lastGyroYaw);
        } else {
          deltaYaw =
              rawGyroRotation.minus(
                  gyroInputs.connected
                      ? gyroInputs.odometryYawPositions[i - 1]
                      : rawGyroRotation.minus(new Rotation2d(twist.dtheta)));
        }
        twist = new Twist2d(twist.dx, twist.dy, deltaYaw.getRadians());

        totalDeltaPose = totalDeltaPose.plus(twist.exp());
        
      } else {
        poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      }
    }

    // Apply update
    if (JsonConstants.featureFlags.useMAPoseEstimator) {
      Twist2d totalTwist = totalDeltaPose.minus(Pose2d.kZero).log();
      lastGyroYaw = gyroInputs.connected ? gyroInputs.yawPosition : rawGyroRotation;
      maPoseEstimator.addDriveData(Timer.getTimestamp(), totalTwist);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    field2d.setRobotPose(getPose());

    long endTimeUs = RobotController.getTime();
    if (JsonConstants.featureFlags.logPeriodicTiming) {
      Logger.recordOutput("PeriodicTime/driveMs", (endTimeUs - startTimeUs) / 1000.0);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisVelocities speeds) {
    // Calculate module setpoints
    ChassisVelocities discreteSpeeds = speeds.discretize(0.02);
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(discreteSpeeds);
    setpointStates = SwerveDriveKinematics.desaturateWheelVelocities(
        setpointStates, JsonConstants.physicalDriveConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void setGoalSpeedsBlueOrigins(ChassisVelocities goalSpeeds) {
    Rotation2d robotRotation = getRotation();

    runVelocity(goalSpeeds.toRobotRelative(robotRotation));
  }

  @Override
  public void setGoalVelocities(ChassisVelocities goalSpeeds, boolean isFieldCentric) {
    Logger.recordOutput("drive/goalSpeeds", goalSpeeds);

    ChassisVelocities speeds = getChassisVelocities();
    if ((xLockPressed || inDefenseMode)
        && Math.abs(goalSpeeds.vx) <= 1e-3
        && Math.abs(goalSpeeds.vy) <= 1e-3
        && Math.abs(goalSpeeds.omega) <= 1e-3
        && Math.sqrt(
                speeds.vx * speeds.vx
                    + speeds.vy * speeds.vy)
            <= JsonConstants.driveConstants.xLockMaxVelocityMetersPerSecond
        && Math.abs(speeds.omega)
            <= JsonConstants.driveConstants.xLockMaxVelocityRadiansPerSecond) {
      stopWithX();
      return;
    }

    if (isFieldCentric) {
      // Adjust for field-centric control
      boolean isFlipped =
          DriverStationBackend.getAlliance().isPresent()
              && DriverStationBackend.getAlliance().get() == Alliance.RED;

      Rotation2d robotRotation =
          isFlipped
              ? getRotation().plus(new Rotation2d(Math.PI)) // Flip orientation for Red Alliance
              : getRotation();

      runVelocity(goalSpeeds.toRobotRelative(robotRotation));
    } else {
      Logger.recordOutput("Drive/DesiredRobotCentricSpeeds", goalSpeeds);

      runVelocity(goalSpeeds);
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    if (JsonConstants.featureFlags.useMAPoseEstimator) {
      return maPoseEstimator.getLatestPose();
    } else {
      return poseEstimator.getEstimatedPosition();
    }
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    if (JsonConstants.featureFlags.useMAPoseEstimator) {
      maPoseEstimator.resetPose(pose);
    } else {
      poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (JsonConstants.featureFlags.useMAPoseEstimator) {
      maPoseEstimator.addVisionData(
          List.of(
              new TimestampedVisionUpdate(
                  timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs)));
    } else {
      poseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return JsonConstants.physicalDriveConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / JsonConstants.physicalDriveConstants.drive_base_radius;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          JsonConstants.physicalDriveConstants.FrontLeft.LocationX,
          JsonConstants.physicalDriveConstants.FrontLeft.LocationY),
      new Translation2d(
          JsonConstants.physicalDriveConstants.FrontRight.LocationX,
          JsonConstants.physicalDriveConstants.FrontRight.LocationY),
      new Translation2d(
          JsonConstants.physicalDriveConstants.BackLeft.LocationX,
          JsonConstants.physicalDriveConstants.BackLeft.LocationY),
      new Translation2d(
          JsonConstants.physicalDriveConstants.BackRight.LocationX,
          JsonConstants.physicalDriveConstants.BackRight.LocationY)
    };
  }

  public void setSteerGains(PIDGains gains) {
    JsonConstants.driveConstants.steerGains = gains;
    for (var module : modules) {
      module.setSteerGains(gains);
    }
  }

  public void setDriveGains(PIDGains gains) {
    JsonConstants.driveConstants.driveGains = gains;
    for (var module : modules) {
      module.setDriveGains(gains);
    }
  }

  /**
   * This sets the supply current limit of all of the swerve modules.
   *
   * @param limit the new current supply limit for the swerves
   */
  public void setSupplyCurrentLimit(Current limit) {
    for (var module : modules) {
      module.setSupplyCurrentLimit(limit);
    }
  }

  /**
   * Seeds the current odometry pose so that the robot is pointed forward (away from the driver
   * station)
   */
  public void seedHeadingForward() {
    Pose2d currentPose = getPose();
    Rotation2d heading =
        switch (AllianceUtil.getAlliance()) {
          // Red alliance is "flipped" (forward is -x)
          case RED -> Rotation2d.k180deg;
          // Blue alliance is +x forward
          case BLUE -> Rotation2d.kZero;
        };

    setPose(new Pose2d(currentPose.getX(), currentPose.getY(), heading));
  }

  /**
   * Sets the defense mode flag.
   *
   * <p>Should be called from the coordination layer when toggling defense mode
   */
  public void setInDefenseMode(boolean inDefenseMode) {
    this.inDefenseMode = inDefenseMode;
  }

  /**
   * Sets the X-lock button pressed flag.
   *
   * <p>Should be called by the coordination layer when the X-lock button is pressed or released
   * with the new state of the button.
   *
   * @param xLockPressed {@code true} when the X-lock button is pressed, {@code false} when it is
   *     not
   */
  public void setXLockPressed(boolean xLockPressed) {
    this.xLockPressed = xLockPressed;
  }
}
