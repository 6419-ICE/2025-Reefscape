package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TestDriveCommand extends FunctionalCommand {
    public TestDriveCommand(DriveSubsystem driveSubsystem) {
        super(()->{},
         ()->driveSubsystem.driveRaw(1, 0, 0, false),
          (bool)->driveSubsystem.driveRaw(0, 0, 0, false),
           ()->false,
            driveSubsystem
        );
    }
}
