// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import org.photonvision.PhotonUtils;

import java.util.Map;

/** An example command that uses an example subsystem. */
public class FollowTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem m_DrivebaseSubsystem;
  private final CameraSubsystem m_CameraSubsystem;

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);


  final double ANGULAR_P = 0.012573;
  final double ANGULAR_D = 0.0;
  final double LINEAR_P = 0.012573;
  final double LINEAR_D = 0.0;

  private final PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  private final PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  // Create Boolean widget that displays vision status
  private final SimpleWidget angularStatusWidget = Shuffleboard.getTab("Vision 2").add("Angular status", false);
  private final SimpleWidget linearStatusWidget = Shuffleboard.getTab("Vision 2").add("Linear status", false);

  private boolean foundTarget;
  private int cyclesEmpty;
  private boolean finishedRunning;

  double rotationSpeed;
  double forwardSpeed;

  double previousTimestamp;

  final double targetRange = Units.feetToMeters(4);

  /**
   * Creates a new AlignRobotToTarget.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowTarget(DrivebaseSubsystem DrivebaseSubsystem, CameraSubsystem CameraSubsystem) {
    m_DrivebaseSubsystem = DrivebaseSubsystem;
    m_CameraSubsystem = CameraSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DrivebaseSubsystem);
    addRequirements(CameraSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    foundTarget = false;
    cyclesEmpty = 0;
    rotationSpeed = 0;
    forwardSpeed = 0;
    
    finishedRunning = false;

    turnController.reset();

    angularStatusWidget.withProperties(Map.of("colorWhenFalse", "grey"));
    linearStatusWidget.withProperties(Map.of("colorWhenFalse", "grey"));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = m_CameraSubsystem.camera.getLatestResult();

    var resultTimestamp = result.getTimestampSeconds();

    if(result.hasTargets()) {
      if(resultTimestamp != previousTimestamp){
        angularStatusWidget.withProperties(Map.of("colorWhenFalse", "yellow"));
        linearStatusWidget.withProperties(Map.of("colorWhenFalse", "yellow"));

        previousTimestamp = resultTimestamp;

        foundTarget = true;

        double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));

        var targetYaw = result.getBestTarget().getYaw();

        rotationSpeed = -turnController.calculate(targetYaw, 0);
        forwardSpeed = -forwardController.calculate(range, targetRange);

        m_DrivebaseSubsystem.set(0, rotationSpeed);

        if(Math.abs(targetYaw) <= 1 && range - targetRange >= Units.feetToMeters(1)) {
          angularStatusWidget.withProperties(Map.of("colorWhenFalse", "lime"));
          linearStatusWidget.withProperties(Map.of("colorWhenFalse", "lime"));
          finishedRunning = true;
        } else if(Math.abs(targetYaw) <= 1) {
          angularStatusWidget.withProperties(Map.of("colorWhenFalse", "lime"));
        } else if(range - targetRange >= Units.feetToMeters(1)) {
          linearStatusWidget.withProperties(Map.of("colorWhenFalse", "lime"));
        }
      }
    } else {
      if(foundTarget) {
        m_DrivebaseSubsystem.set(0, rotationSpeed);
        cyclesEmpty++;

        if(cyclesEmpty >= 35) {
          angularStatusWidget.withProperties(Map.of("colorWhenFalse", "cyan"));
          finishedRunning = true;
        }
      } else {
        angularStatusWidget.withProperties(Map.of("colorWhenFalse", "red"));
        finishedRunning = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      angularStatusWidget.withProperties(Map.of("colorWhenFalse", "grey"));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedRunning;
  }
}