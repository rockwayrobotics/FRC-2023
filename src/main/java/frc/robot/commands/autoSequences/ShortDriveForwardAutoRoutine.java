package frc.robot.commands.autoSequences;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.FailFastTimeoutGroup;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Map;


/** Autonomously shoots the piece loaded in auto, and drives outside the short side of the community
 * <p><strong>SETUP:</strong> Place the robot on short side of community beside charge station
 * <p><strong>END:</strong> The robot has driven beyond the line to get out of the community, but not too far beyond the line
 * <p><strong>SCORES:</strong> Piece in auto, Auto mobility
 */
public class ShortDriveForwardAutoRoutine extends SequentialCommandGroup {
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

    public ShortDriveForwardAutoRoutine(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, LedSubsystem led, SimpleWidget AutoFailedWidget) {
        m_drivebase = drivebase;
        m_drivebase.setAutoOffset(90);

        m_shooter = shooter;

        m_led = led;

        AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "yellow"));

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
                .thenWithTimeout(new DriveDistance(drivebase, 0.2, 2), 15);


        this.addCommands(sequence);
        this.addCommands(new InstantCommand(() -> setStatusWidget(AutoFailedWidget, sequence)));
    }
}