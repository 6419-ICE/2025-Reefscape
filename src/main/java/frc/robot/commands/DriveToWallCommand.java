package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToWallCommand extends RunCommand {
    private DriveSubsystem driveSubsystem;
    public DriveToWallCommand(DriveSubsystem driveSubsystem) {
        super(()->driveSubsystem.drive(0.25, 0, 0, false), driveSubsystem);
        this.driveSubsystem = driveSubsystem;
    }
    @Override
    public boolean isFinished() {
        return driveSubsystem.getAverageDriveCurrentDraw() >= ModuleConstants.currentDrawAgainstWall;
    }
    
}
