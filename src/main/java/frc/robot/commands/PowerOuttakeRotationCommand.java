package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.OuttakeSubsystem;

public class PowerOuttakeRotationCommand extends RunCommand {
    public PowerOuttakeRotationCommand(double power, OuttakeSubsystem outtakeSubsystem) {
        super(()->outtakeSubsystem.setRotationPower(power));
    }
}
