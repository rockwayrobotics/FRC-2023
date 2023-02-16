// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DriveDistance;

public class RobotContainer {

  private final DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem();

  private final XboxController m_xboxController = new XboxController(Gamepads.kXboxControllerID);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public final GenericEntry autoSpeed;

  public final Command m_driveForever;
  public final Command m_driveForeverSlow;

  // Create a voltage constraint to ensure we don't accelerate too fast
  final DifferentialDriveVoltageConstraint autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          Constants.Drivetrain.kS,
          Constants.Drivetrain.kV,
          Constants.Drivetrain.kA),
      Constants.Drivetrain.kDriveKinematics,
      10);

  // Create config for trajectory
  TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.Auto.kMaxSpeedMetersPerSecond,
            Constants.Auto.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.Drivetrain.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

  RamseteCommand ramseteCommand =
    new RamseteCommand(
        exampleTrajectory,
        m_drivebase::getPose,
        new RamseteController(Constants.Auto.kRamseteB, Constants.Auto.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.Drivetrain.kS,
            Constants.Drivetrain.kV,
            Constants.Drivetrain.kA),
        Constants.Drivetrain.kDriveKinematics,
        m_drivebase::getWheelSpeeds,
        new PIDController(Constants.Drivetrain.kP, 0, 0),
        new PIDController(Constants.Drivetrain.kP, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivebase::tankDriveVolts,
        m_drivebase);

  public RobotContainer() {
    var autoTab = Shuffleboard.getTab("Auto");
    autoSpeed = autoTab.addPersistent("Max Speed", 1).withPosition(2, 0).getEntry();
    
    m_driveForever = new DriveDistance(m_drivebase, () -> autoSpeed.getDouble(0.5));
    m_driveForeverSlow = new DriveDistance(m_drivebase, () -> 0.1);

    m_chooser.setDefaultOption("Drive Forever", m_driveForever);
    m_chooser.addOption("Drive Forever Slow", m_driveForeverSlow);
    autoTab.add("Auto Routine", m_chooser).withSize(2, 1).withPosition(0, 0);

    m_drivebase.setDefaultCommand(new DriveCommand(m_xboxController::getLeftY, m_xboxController::getRightX, m_drivebase));

    Shuffleboard.getTab("Subsystems").add(m_drivebase);

    configureBindings();
  }

  private void configureBindings() {
    final JoystickButton rightBumper = new JoystickButton(m_xboxController, XboxController.Button.kLeftBumper.value);
    rightBumper.onTrue(new SetDriveScaleCommand(m_drivebase, Drivetrain.kSlowmodeScale));
    rightBumper.onFalse(new SetDriveScaleCommand(m_drivebase, 1));
  }

  public Command getAutonomousCommand() {
    // Reset odometry to the starting pose of the trajectory.
    m_drivebase.resetOdometry(exampleTrajectory.getInitialPose());

    m_chooser.addOption("Ramsete command", ramseteCommand.andThen(() -> m_drivebase.tankDriveVolts(0, 0)));

    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
}
