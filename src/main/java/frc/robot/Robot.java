// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();

  // fields that adjust the response for manual driving
  private boolean fieldRelative = true;
  private boolean invertControls = true;
  private double speedMultiplier = 0.5; // factor applied to joystick drive commands

  // track alliance reported by driverstaion
  @NotLogged private Alliance prevAlliance = null;

  // Driver controller
  private CommandXboxController driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  public Robot() {
    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    DataLogManager.start();

    // This will log the joysticks & control data from the Driver Station
    DriverStation.startDataLog(DataLogManager.getLog());

    Epilogue.bind(this);

    var backend = Epilogue.getConfig().backend;
    backend.log("Robot/buildInfo/currentBranch", BuildConstants.GIT_BRANCH);
    backend.log("Robot/buildInfo/buildDate", BuildConstants.BUILD_DATE);
    backend.log("Robot/buildInfo/commitId", BuildConstants.GIT_SHA);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController.x().whileTrue(robotDrive.setXCommand());
    driverController.back().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));
    driverController
        .a()
        .onTrue(robotDrive.runOnce(() -> robotDrive.zeroHeading(robotDrive.getPose())));
    driverController.start().onTrue(new InstantCommand(() -> resetRobotToFieldCenter()));
  }

  /** Use this method to define default commands for subsystems. */
  private void configureDefaultCommands() {
    robotDrive.setDefaultCommand(
        robotDrive.driveCommand(
            adjustJoystick(
                driverController::getLeftY,
                () -> speedMultiplier,
                () -> invertControls || !fieldRelative),
            adjustJoystick(
                driverController::getLeftX,
                () -> speedMultiplier,
                () -> invertControls || !fieldRelative),
            adjustJoystick(driverController::getRightX, () -> speedMultiplier, () -> true),
            () -> fieldRelative));
  }

  /**
   * Use this to select the autonomous command.
   *
   * @return the command to run in autonomous
   */
  private Command getAutonomousCommand() {
    return new InstantCommand();
  }

  /**
   * Apply desired adjustments to a joystick input, such as deadbanding and nonlinear transforms.
   *
   * @param input The input value from the joystick
   * @param negate Whether to invert the input
   * @return The adjusted value from the joystick
   */
  private DoubleSupplier adjustJoystick(
      DoubleSupplier input, DoubleSupplier multiplier, BooleanSupplier negate) {
    return () -> {
      double value = input.getAsDouble();
      if (negate.getAsBoolean()) {
        value = -value;
      }
      value = MathUtil.applyDeadband(value, OIConstants.kDriveDeadband);
      value = multiplier.getAsDouble() * value;
      return value;
    };
  }

  public void resetRobotToFieldCenter() {
    var field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    var heading =
        (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red)
            ? 180.0
            : 0.0;
    robotDrive.zeroHeading();
    robotDrive.resetOdometry(
        new Pose2d(
            field.getFieldLength() / 2,
            field.getFieldWidth() / 2,
            Rotation2d.fromDegrees(heading)));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (!DriverStation.getAlliance().isEmpty()) {
      var alliance = DriverStation.getAlliance().get();
      invertControls = isSimulation() || alliance.equals(Alliance.Blue);
      if (prevAlliance == null || !prevAlliance.equals(alliance)) {
        resetRobotToFieldCenter();
        prevAlliance = alliance;
      }
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
