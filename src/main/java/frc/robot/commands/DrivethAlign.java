// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Map;
import java.util.List;

import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;

import frc.robot.Projectile_Math.ShotConfig;
import frc.robot.Projectile_Math;
import frc.robot.Constants.Shooter;
import frc.robot.Target;

public class DrivethAlign extends CommandBase {
  /** Creates a new DrivethAlign. */
  private final DrivebaseSubsystem m_DrivebaseSubsystem;
  private final CameraSubsystem m_CameraSubsystem;
  private final Projectile_Math m_ProjectileMath;

  private PIDController turnController;
  private PIDController driveController;

  private SimpleWidget colorWidget;

  double rotationSpeed;
  double driveSpeed;

  double previousDistance;
  double startingDistance;
  double desiredDistance;

  Direction direction;


  public DrivethAlign(DrivebaseSubsystem DrivebaseSubsystem, CameraSubsystem CameraSubsystem) {
    m_DrivebaseSubsystem = DrivebaseSubsystem;
    m_CameraSubsystem = CameraSubsystem;
    m_ProjectileMath = new Projectile_Math();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DrivebaseSubsystem);
    addRequirements(CameraSubsystem);

    //these are likely too high, you will need to tune them likely
    final double ANGULAR_P = 0.75;
    final double ANGULAR_D = 0.0;

    turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    driveController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
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

  private PhotonTrackedTarget get_desired_target(List<PhotonTrackedTarget> target_list, int id){
    for (PhotonTrackedTarget target : target_list ){
      // System.out.println("ID: " + target.getFiducialId());
      if (target.getFiducialId() == id){
        return target;
      }
    }
    return null;
  }

  /**
   * Using the camera and April Tag finds the directions needed to complete a shot.
   * If no April Tags are found, the Direction.ok will be False.
   * @return Direction
   */
  private Direction get_directions(){
    //Eventaully will be given dynamically
    int id = 0;
    PhotonPipelineResult result = m_CameraSubsystem.camera.getLatestResult();
    double angle_thresh = 0.1; //in radians
    double x = 0;
    double y = 0;
    double angle = 0;
    boolean ok = result.hasTargets();
    if (ok){
      PhotonTrackedTarget target = get_desired_target(result.getTargets(), id);
      if (target != null){
        Transform3d three_d = target.getBestCameraToTarget();
        x = three_d.getX();
        y = three_d.getY();
        angle = Math.atan(y/x);
        System.out.println("xyz: " + x + " " + y + " " + Units.radiansToDegrees(angle));
      }

    }
    if (angle < angle_thresh){
      angle = 0;
    }
    return new Direction(x, y, angle, ok);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("bean");
    turnController.reset();
    driveController.reset();
    previousDistance = 10;
    direction = get_directions();
    m_DrivebaseSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Needs to get dynamically passed at some point
    Target target = Shooter.MID_CONE;
    //Threshold in meters of the y distance driven (must be able to hit a target within this tolerance side to side)
    double distanceThresh = 0.0; //may need to make this larger
    double currentDistance;
    //Checks if a target is visible
    if (direction.ok){
      //checks if the angle is 0, if it has not, continues its rotation
      // if (direction.angle != 0){
      //   rotationSpeed = turnController.calculate(direction.angle, 0);
      //   m_DrivebaseSubsystem.set(0, rotationSpeed);
      // }
      //checks if the robot has arrived at its destination, if it has not, continues driving
      if (previousDistance != 0){
        // currentDistance = direction.y + target.side_offset;

        //must be square for this to make any sense
        currentDistance = direction.y + m_DrivebaseSubsystem.getAverageDistance();
        driveSpeed = -turnController.calculate(currentDistance, target.side_offset);
        System.out.println("Lets go just this amount more: " + currentDistance);
        m_DrivebaseSubsystem.set(driveSpeed,0);
        previousDistance = currentDistance;
      }
    }
    else{
      System.out.println("No Target Has Been Spotted!");
    }


    //Code that deals with aiming
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
