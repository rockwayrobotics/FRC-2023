// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import frc.robot.Constants;
import frc.robot.commands.autoSequences.BalanceRoutine;
import frc.robot.commands.autoSequences.CommunityRoutine;
import frc.robot.commands.autoSequences.LongDriveForwardAutoRoutine;
import frc.robot.commands.autoSequences.ShortDriveForwardAutoRoutine;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Map;

public class CameraSubsystem extends SubsystemBase {
  public static PhotonCamera camera;

  ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");

  HttpCamera rawCamera = new HttpCamera(Constants.Vision.camName + " - Raw", "http://10.80.89.3:1181/stream.mjpg");
  HttpCamera processedCamera = new HttpCamera(Constants.Vision.camName + " - Processed", "http://10.80.89.3:1182/stream.mjpg");

  // TODO Rewrite to be more variable
  SuppliedValueWidget<Boolean> m_distanceWidget = Shuffleboard.getTab("Dashboard").addBoolean("TURN", this::getInDistance)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withProperties(Map.of("colorWhenFalse", "red", "colorWhenTrue", "green"));

  boolean getInDistance() {
//    System.out.println("Updating");

    var vision = CameraSubsystem.camera.getLatestResult();
    if (!vision.hasTargets()) {
      return false;
    }
    return vision.getBestTarget().getBestCameraToTarget().getX() < 5;
  }

  public CameraSubsystem() {
    CameraServer.addCamera(rawCamera);
    CameraServer.addCamera(processedCamera);
    dashboard.add(rawCamera).withPosition(4, 0).withSize(3, 3);

    camera = new PhotonCamera(Constants.Vision.camName);

    dashboard.add("Distance", bestTargetX()).withPosition(4,3);
    dashboard.add("Horizontal", bestTargetY()).withPosition(5,3);
    dashboard.add("Skew", bestTargetSkew()).withPosition(6,3);
    dashboard.add("Valid alliance target", getBestAllianceTarget() != null).withPosition(7, 3);
  }

  public List<PhotonTrackedTarget> getTargetList() {
    PhotonPipelineResult cameraResult = camera.getLatestResult();
    if(cameraResult.hasTargets()) {
      return cameraResult.getTargets();
    } else {
      return null;
    }
  }

  public PhotonTrackedTarget getBestAllianceTarget() {
    PhotonPipelineResult cameraResult = camera.getLatestResult();
    PhotonTrackedTarget myResult = null;

    if(cameraResult.hasTargets()) {
      if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        switch (cameraResult.getBestTarget().getFiducialId()) {
          case 4, 6, 7, 8 -> myResult = cameraResult.getBestTarget();
        };
      } else if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        switch (cameraResult.getBestTarget().getFiducialId()) {
          case 1,2,3,4 -> myResult = cameraResult.getBestTarget();
        };
      }
    }

    return myResult;
  }

  public double bestTargetX() {
    if(getBestAllianceTarget() != null) {
      return getBestAllianceTarget().getBestCameraToTarget().getX();
    } else {
      return 0;
    }
  }

  public double bestTargetY() {
    if(getBestAllianceTarget() != null) {
      return getBestAllianceTarget().getBestCameraToTarget().getY();
    } else {
      return 0;
    }
  }

  public double bestTargetSkew() {
    if(getBestAllianceTarget() != null) {
      Transform3d targetTransform = getBestAllianceTarget().getBestCameraToTarget();
      return Math.atan2(targetTransform.getX(), targetTransform.getY());
    } else {
      return 0;
    }
  }

  public static PhotonTrackedTarget get_desired_target(List<PhotonTrackedTarget> target_list, int aprilTagID){
    for (PhotonTrackedTarget target : target_list ){
      if (target.getFiducialId() == aprilTagID){
        return target;
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    PhotonTrackedTarget myTarget = getBestAllianceTarget();
    if(myTarget != null) {
      Transform3d bestTarget = myTarget.getBestCameraToTarget();

      double distance = bestTarget.getX();
      double horizontalTranslation = bestTarget.getY();
      double skew = Math.atan2(distance, horizontalTranslation);


    }

  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}
