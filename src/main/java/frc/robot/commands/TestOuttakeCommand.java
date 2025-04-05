package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OuttakeAngles;
import frc.robot.subsystems.OuttakeSubsystem;

public class TestOuttakeCommand extends SequentialCommandGroup {
    public TestOuttakeCommand(OuttakeSubsystem outtakeSubsystem) {
        addCommands(
            new RotateOuttakeCommand(OuttakeAngles.intake, outtakeSubsystem),
            new WaitCommand(5),
            new RotateOuttakeCommand(OuttakeAngles.topScore, outtakeSubsystem)
        );
    }
    
}
