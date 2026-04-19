// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Logged
public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule[] swerveModules = {
    new MAXSwerveModule(
        kFrontLeftDrivingCanId, kFrontLeftTurningCanId, kFrontLeftChassisAngularOffset),
    new MAXSwerveModule(
        kFrontRightDrivingCanId, kFrontRightTurningCanId, kFrontRightChassisAngularOffset),
    new MAXSwerveModule(
        kRearLeftDrivingCanId, kRearLeftTurningCanId, kBackLeftChassisAngularOffset),
    new MAXSwerveModule(
        kRearRightDrivingCanId, kRearRightTurningCanId, kBackRightChassisAngularOffset)
  };

  // The gyro sensor
  private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final SimDeviceSim navxSim = new SimDeviceSim("navX-Sensor", navx.getPort());
  private final SimDouble navxSimAngle = navxSim.getDouble("Yaw");

  // Odometry class for tracking robot pose
  private final SwerveDrivePoseEstimator odometry =
      new SwerveDrivePoseEstimator(
          kDriveKinematics, Rotation2d.kZero, getPositions(), Pose2d.kZero);

  // Used for logging swerve drive states
  @SuppressWarnings("unused")
  private ChassisSpeeds speedsMeasured = new ChassisSpeeds();

  private ChassisSpeeds speedsRequested = new ChassisSpeeds();
  private SwerveModuleState[] statesRequested =
      kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds());
  private SwerveModuleState[] statesMeasured = statesRequested;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  private SwerveModulePosition[] getPositions() {
    var positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPosition();
    }
    return positions;
  }

  private SwerveModuleState[] getStates() {
    var states = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      states[i] = swerveModules[i].getState();
    }
    return states;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(navx.getRotation2d(), getPositions());

    statesMeasured = getStates();
    speedsMeasured = kDriveKinematics.toChassisSpeeds(statesMeasured);
  }

  @Override
  public void simulationPeriodic() {
    double timestep = 20e-3;
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].simulationPeriodic(timestep);
    }
    double dTheta = (speedsRequested.omegaRadiansPerSecond * timestep) * 180 / Math.PI;
    navxSimAngle.set(navxSimAngle.get() - dTheta);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @Logged(name = "Pose")
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    odometry.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPosition(Pose2d pose) {
    odometry.resetPosition(navx.getRotation2d(), getPositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * kMaxAngularSpeed;

    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, getPose().getRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    setModuleStates(kStatesX);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeedMetersPerSecond);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(desiredStates[i]);
    }

    statesRequested = desiredStates;
    speedsRequested = kDriveKinematics.toChassisSpeeds(desiredStates);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].resetEncoders();
    }
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.zeroYaw();
  }

  /**
   * Zeroes the heading of the robot and sets the pose.
   *
   * @param pose The pose that the robot will have after reset.
   */
  public void setPose(Pose2d pose) {
    zeroHeading();
    resetPosition(pose);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  @Logged(name = "Heading (degrees)")
  public double getHeading() {
    return MathUtil.inputModulus(getAngle(), -180, 180);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  @Logged(name = "TurnRate (degrees per second)")
  public double getTurnRate() {
    return navx.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the unwrapped angle of the robot.
   *
   * @return The unwrapped angle of the robot, in degrees
   */
  @Logged(name = "Angle (degrees)")
  public double getAngle() {
    return navx.getAngle() * (kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Command for driving the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public Command driveCommand(
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rot,
      BooleanSupplier fieldRelative) {
    return run(
        () ->
            drive(
                xSpeed.getAsDouble(),
                ySpeed.getAsDouble(),
                rot.getAsDouble(),
                fieldRelative.getAsBoolean()));
  }

  /** Command to set the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return run(this::setX).withName("Lock Wheels");
  }
}
