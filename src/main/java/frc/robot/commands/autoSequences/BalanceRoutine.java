package frc.robot.commands.autoSequences;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/** Autonomously drives over the charge station, then reverses, and balances itself on top.
 * <p><strong>SETUP:</strong> Place front of robot directly facing charge station. Be sure to be far enough away that it can get the
 * speed it needs,but not too far that it takes to long to get there.
 * <p><strong>END:</strong> The robot is balanced on top of the charge station
 * <p><strong>SCORES:</strong> Piece in auto, Auto mobility, Auto engaged
 */
public class BalanceRoutine extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;
    ShooterSubsystem m_shooter;
    LedSubsystem m_led;

    public void setStatusWidget(SimpleWidget AutoFailedWidget, FailFastTimeoutGroup sequence) {
        if(sequence.timedOut()) {
            AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "red"));

        } else {
            AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "green"));
        }
    }

    public BalanceRoutine(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, LedSubsystem led, SimpleWidget AutoFailedWidget) {
        m_drivebase = drivebase;
        m_drivebase.setAutoOffset(90);

        m_shooter = shooter;

        m_led = led;

        AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "yellow"));
        m_led.setMode(Constants.LED.modes.Purple);

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
            .then(new AutoShootSequence(m_drivebase, m_shooter, m_led))
            .then(new DriveUntilTipped(drivebase, -12, 0.4))
            // .then(new DriveUntilTipped(drivebase, 16, 0.4))
            // .then(new DriveUntilTipped(drivebase, 3, 0.2))
            // .then(new DriveDistance(drivebase, 0.3, 20))
            // .then(new WaitCommand(0.7))
            // .then(new DriveUntilTipped(drivebase, 16, -0.6))
            .then(new AutoBalance(drivebase))
            .then(new WaitCommand(1))
            .then(new AutoBalance(drivebase));

        // this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Orange)));
        // this.addCommands(new AutoShootSequence(m_drivebase, m_shooter, m_led));
        // this.addCommands(new DriveUntilTipped(drivebase, -12, 0.4));
        // this.addCommands(new DriveUntilTipped(drivebase, 16, 0.4));
        // this.addCommands(new DriveUntilTipped(drivebase, 3, 0.2));
        // this.addCommands(new DriveDistance(drivebase, 0.2, 30));
        // System.out.println("balanceTime yipeee bing bang bong boom");
        // this.addCommands(new WaitCommand(0.7));
        // this.addCommands(new DriveUntilTipped(drivebase, 16, -0.5));
        // this.addCommands(new AutoBalance(drivebase));
        // this.addCommands(new WaitCommand(0.5));
        // this.addCommands(new AutoBalance(drivebase));

        this.addCommands(sequence);
        this.addCommands(new InstantCommand(() -> setStatusWidget(AutoFailedWidget, sequence)));
    }
}