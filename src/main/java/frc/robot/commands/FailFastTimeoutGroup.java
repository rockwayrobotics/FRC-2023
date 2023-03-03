package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FailFastTimeoutGroup extends SequentialCommandGroup {
    boolean m_timedOut = false;

    FailFastTimeoutGroup() {}

    public FailFastTimeoutGroup thenWithTimeout(Command command, double timeout) {
        this.addCommands(new ConditionalCommand(
            new InstantCommand(() -> {}),
            command.finallyDo(this::commandEnd).withTimeout(timeout),
            this::timedOut
        ));
        return this;
    }

    public FailFastTimeoutGroup then(Command command) {
        this.addCommands(command);
        return this;
    }

    void commandEnd(boolean cancelled) {
        if (cancelled) {
            m_timedOut = true;
        }
    }

    public boolean timedOut() {
        return m_timedOut;
    }
}
