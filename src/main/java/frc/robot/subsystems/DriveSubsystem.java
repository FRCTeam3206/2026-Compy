// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Logged
public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule[] swerveModules = {
    new MAXSwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset),
    new MAXSwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset),
    new MAXSwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset),
    new MAXSwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset)
  };

  // The gyro sensor
  private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final SimDeviceSim navxSim = new SimDeviceSim("navX-Sensor", navx.getPort());
  private final SimDouble navxSimAngle = navxSim.getDouble("Yaw");

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d.kZero, getPositions());

  private SwerveModuleState[] statesRequested =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds());
  private SwerveModuleState[] statesMeasured = statesRequested;
  private ChassisSpeeds speedsRequested =
      DriveConstants.kDriveKinematics.toChassisSpeeds(statesRequested);

  @SuppressWarnings("unused")
  private ChassisSpeeds speedsMeasured = speedsRequested;

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

  // for logging
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
    speedsMeasured = DriveConstants.kDriveKinematics.toChassisSpeeds(statesMeasured);
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
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
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
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, getPose().getRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    setModuleStates(DriveConstants.kStatesX);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(desiredStates[i]);
    }

    statesRequested = desiredStates;
    speedsRequested = DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates);
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
    resetOdometry(pose);
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
    return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the unwrapped angle of the robot.
   *
   * @return The unwrapped angle of the robot, in degrees
   */
  @Logged(name = "Angle (degrees)")
  public double getAngle() {
    return navx.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
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
