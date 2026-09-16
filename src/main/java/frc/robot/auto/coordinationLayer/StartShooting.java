package frc.robot.auto.coordinationLayer;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import frc.robot.auto.AutoAction;

public class StartShooting extends AutoAction {

  @Override
  public Command toCommand(AutoActionContext data) {
    return new InstantCommand(data.coordinationLayer()::startShootingForAuto);
  }
}
