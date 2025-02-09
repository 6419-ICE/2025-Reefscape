package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HangerAngles;
import frc.robot.subsystems.HangerSubsystem;

public class MoveHangerCommand extends Command {
    private HangerSubsystem hangerSubsystem;
    private HangerAngles angle;
    public MoveHangerCommand(HangerAngles angle, HangerSubsystem hangerSubsystem) {
        this.hangerSubsystem = hangerSubsystem;
        this.angle = angle;
    }
    @Override
    public void initialize() {
        hangerSubsystem.setAngle(angle.getValue());
    }
    @Override
    public boolean isFinished() {
        return hangerSubsystem.atAngle();
    }
}
