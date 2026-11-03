package frc.robot.auto.coordinationLayer;

import org.wpilib.command2.Command;
import frc.robot.auto.AutoAction;

public class ClimbSearchAction extends AutoAction {

  @Override
  public Command toCommand(AutoActionContext data) {
    return new Command() {
      @Override
      public void execute() {
        data.coordinationLayer().climbSearchForAuto();
      }

      @Override
      public boolean isFinished() {
        return data.coordinationLayer().isClimbSearchFinishedForAuto();
      }
    };
  }
}
