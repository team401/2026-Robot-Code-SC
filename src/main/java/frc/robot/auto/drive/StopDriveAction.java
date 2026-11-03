package frc.robot.auto.drive;

import org.wpilib.command2.Command;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;

public class StopDriveAction extends DriveAutoAction {

  @Override
  public Command toCommand(AutoActionContext data) {
    return DriveCoordinatorCommands.wrapCommand(
        data.driveCoordinator(), DriveCoordinatorCommands.stopDrive(data.driveCoordinator()));
  }
}
