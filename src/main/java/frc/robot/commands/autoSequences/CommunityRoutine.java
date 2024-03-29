package frc.robot.commands.autoSequences;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/** Autonomously drives over the charge station, and stops on the opposite side
 * <p><strong>SETUP:</strong> Place front of robot directly facing charge station. Be sure to be far enough away that it can get the
 * speed it needs,but not too far that it takes to long to get there.
 * <p><strong>END:</strong> The robot is sitting on the opposite side of the charge station outside the community
 * <p><strong>SCORES:</strong> Piece in auto, Auto mobility
 */
public class CommunityRoutine extends SequentialCommandGroup {
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

    public CommunityRoutine(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, LedSubsystem led, SimpleWidget AutoFailedWidget) {
        m_drivebase = drivebase;
        m_drivebase.setAutoOffset(90);

        m_shooter = shooter;

        m_led = led;

        AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "yellow"));

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
                .thenWithTimeout(new AutoShootSequence(m_drivebase, m_shooter, m_led), 15)
                .thenWithTimeout(new DriveUntilTipped(drivebase, -12, 0.4), 15)
                .thenWithTimeout(new DriveUntilTipped(drivebase, 12, 0.2), 15)
                .thenWithTimeout(new DriveUntilTipped(drivebase, 0, 0.2), 15)
                .thenWithTimeout(new DriveDistance(drivebase, 0.2, 30), 15);


        this.addCommands(sequence);
        this.addCommands(new InstantCommand(() -> setStatusWidget(AutoFailedWidget, sequence)));
    }
}