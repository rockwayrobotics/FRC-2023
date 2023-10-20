package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootSequence extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;
    ShooterSubsystem m_shooter;
    LedSubsystem m_led;

    public ShootSequence(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, LedSubsystem led) {
        m_drivebase = drivebase;
        m_shooter = shooter;
        m_led = led;

        this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Red)));

        // Take over drivebase
        //this.addCommands((new ProxyCommand(new ShootMoveDistance(drivebase, 0.2))));
        //this.addCommands(new ShootMoveDistance(drivebase, .2));

        // this.addCommands(new ShootAngle(m_drivebase, m_shooter, 1));

        this.addCommands(new InstantCommand(() -> m_shooter.setFlap(DoubleSolenoid.Value.kForward)));
        this.addCommands(new WaitCommand(0.4));
        this.addCommands(new InstantCommand(() -> m_shooter.setBucketCylinders(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward)));
        // Start driving here
        this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Green)));
        this.addCommands(new WaitCommand(.5));
        this.addCommands(new InstantCommand(() -> m_shooter.setBucketCylinders(DoubleSolenoid.Value.kReverse, DoubleSolenoid.Value.kReverse)));
        this.addCommands(new WaitCommand(1));
        this.addCommands(new InstantCommand(() -> m_shooter.setFlap(DoubleSolenoid.Value.kReverse)));
        this.addCommands(new SetLEDAfterShot(shooter, led));
        // this.addCommands(new BucketToZero(shooter, 0.8));
    }
}