package frc.robot.auto.general;

import org.wpilib.units.measure.Time;
import org.wpilib.command2.Command;
import org.wpilib.command2.WaitCommand;
import frc.robot.auto.AutoAction;

public class Wait extends AutoAction {
  public Time delay;

  @Override
  public Command toCommand(AutoActionContext data) {
    return new WaitCommand(delay);
  }
}
