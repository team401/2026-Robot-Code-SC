package frc.robot.auto.general;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import frc.robot.auto.AutoAction;

// This will allow us to print messages to the console during auto to help with debugging
public class Print extends AutoAction {
  public String message;

  @Override
  public Command toCommand(AutoActionContext data) {
    return new InstantCommand(
        () -> {
          System.out.println(message);
        });
  }
}
