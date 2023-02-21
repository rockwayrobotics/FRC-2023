// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Map;

import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;

import frc.robot.Projectile_Math.ShotConfig;
import frc.robot.Projectile_Math;
import frc.robot.Constants.Shooter;

public class DrivethAlign extends CommandBase {
  /** Creates a new DrivethAlign. */
  private final DrivebaseSubsystem m_DrivebaseSubsystem;
  private final CameraSubsystem m_CameraSubsystem;
  private final Projectile_Math m_ProjectileMath;

  private PIDController turnController;

  private SimpleWidget colorWidget;

  private boolean foundTarget;
  private int cyclesEmpty;
  private boolean finishedRunning;

  double rotationSpeed;

  double previousTimestamp;

  public DrivethAlign(DrivebaseSubsystem DrivebaseSubsystem, CameraSubsystem CameraSubsystem) {
    m_DrivebaseSubsystem = DrivebaseSubsystem;
    m_CameraSubsystem = CameraSubsystem;
    m_ProjectileMath = new Projectile_Math();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DrivebaseSubsystem);
    addRequirements(CameraSubsystem);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;

    turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  }

  /**
   * Class that holds the directions that need to be followed to complete a shot.
   */
  private static class Direction {
    double x;
    double y;
    double angle;
    boolean ok;

    public Direction(double x1, double y1, double angle1, boolean ok1) {
      x = x1;
      y = y1;
      angle = angle1;
      ok = ok1;
    }

    @Override
    public String toString(){
        return "X: " + x + " Y: " + y + "\nAngle: " + angle + " OK: " + ok;
    }

  }

  /**
   * Using the camera and April Tag finds the directions needed to complete a shot.
   * If no April Tags are found, the Direction.ok will be False.
   * @return Direction
   */
  private Direction get_directions(){
    var result = m_CameraSubsystem.camera.getLatestResult();
    double x = 0;
    double y = 0;
    double angle = 0;
    boolean ok = result.hasTargets();
    if (ok){
      var target = result.getBestTarget();
      Transform3d three_d = target.getBestCameraToTarget();
      x = three_d.getX();
      y = three_d.getY();
      angle = Math.atan(y/x);
      System.out.println("xyz: " + x + " " + y + " " + Units.radiansToDegrees(angle));
    }
    return new Direction(x, y, angle, ok);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("bean");
    turnController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Direction direction = get_directions();
    // System.out.println(direction);
    // var result = m_CameraSubsystem.camera.getLatestResult();
    // if (result.hasTargets()){
    // var target = result.getBestTarget();
    // Transform3d three_d = target.getBestCameraToTarget();
    // double x = three_d.getX();
    // double y = three_d.getY();
    // double z = three_d.getZ();
    // ShotConfig a = m_ProjectileMath.aim(Shooter.MID_CUBE, x);
    // System.out.println(x);
    // System.out.println(a);
    // // System.out.println("The Pose is: " + three_d);
    // double range = PhotonUtils.calculateDistanceToTargetMeters(
    // Units.inchesToMeters(7),
    // Units.inchesToMeters(23),
    // 0,
    // Units.degreesToRadians(result.getBestTarget().getPitch()));
    // System.out.println("The Range is: " + range);
    // System.out.println(three_d.getX());
    // System.out.println(three_d.getY());
    // System.out.println(three_d.getZ());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
