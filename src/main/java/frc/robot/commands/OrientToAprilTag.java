package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class OrientToAprilTag extends Command {
    private LimelightSubsystem limelightSubsystem;
    private DriveSubsystem driveSubsystem;
    private double xDiff,zDiff,angle;
    public OrientToAprilTag(double xDiff, double zDiff, double angle, LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.xDiff = xDiff;
        this.zDiff = zDiff;
        this.angle = angle;
    }
    public OrientToAprilTag(int level, boolean left, LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
        this(level == 1 ? 0 : LimelightConstants.reefXDiff * (left ? -1 : 1), LimelightConstants.reefZDiff, 0, limelightSubsystem, driveSubsystem);
    }
    @Override
    public void execute() {
        driveSubsystem.driveRaw(
            LimelightConstants.translationController.calculate(limelightSubsystem.get3dZOffset(),zDiff), 
            LimelightConstants.translationController.calculate(limelightSubsystem.get3dXOffset(),xDiff), 
            LimelightConstants.rotationController.calculate(limelightSubsystem.get3dYawOffset(),angle), 
            false
        );
    }
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveRaw(0, 0, 0, false);
    }
}
