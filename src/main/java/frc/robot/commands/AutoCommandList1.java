package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

import frc.robot.commands.DriveDistance;

//Commands added here will be run in sequence when AutonomousCmdList is called upon 
public class AutoCommandList1 extends SequentialCommandGroup {
  /**
   * A command that spins the wheels for a certain distance
   * @param m_drivebase Set this to m_drivebase
   * @param m_shooter Set this to m_shooter
   * @param m_feeder Set this to m_feeder
   * @param drivedistance Set to distance in inches
   * @param driveSpeed Set to speed from -1 to 1 (must match sign of distance)
   */
    public AutoCommandList1(DrivebaseSubsystem m_drivebase, DoubleSupplier driveDistance) {
        super();
        this.addCommands(new DriveDistance(m_drivebase, driveDistance, () -> 0.3));

    }
}
