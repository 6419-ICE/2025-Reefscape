package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorAndOuttakePositions;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OuttakeStates;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class FullOuttakeCommand extends SequentialCommandGroup {
    public FullOuttakeCommand(int level, boolean left, OuttakeSubsystem outtakeSubsystem, ElevatorSubsystem elevatorSubsystem, LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
        if (level < 0 || level > 4) DriverStation.reportError("Level must be 1,2,3, or 4",true);
        addCommands(
            new OrientToAprilTagCommand(level, left, limelightSubsystem, driveSubsystem),
            Commands.parallel(
                new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.valueOf("L" + level), elevatorSubsystem, outtakeSubsystem),
                //drive until its against the wall
                new DriveToWallCommand(driveSubsystem)
            ),
            new RunOuttakeCommand(OuttakeStates.outtake, outtakeSubsystem).withTimeout(1),
            new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.inside, elevatorSubsystem, outtakeSubsystem)
        );
    }
    
}
