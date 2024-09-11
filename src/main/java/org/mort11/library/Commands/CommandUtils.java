package org.mort11.library.Commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandUtils {

    public Trigger doubleInput (Trigger trigger1, Trigger trigger2) {
        return trigger1.and(trigger2);
    }

    public Trigger tripleInput(Trigger trigger1, Trigger trigger2, Trigger trigger3) {
        return trigger1.and(trigger2.and(trigger3));
    }
}
