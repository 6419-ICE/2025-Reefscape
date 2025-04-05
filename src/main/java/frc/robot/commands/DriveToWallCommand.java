package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToWallCommand extends Command {
    private Command cmd;
    private DriveSubsystem driveSubsystem;
    public DriveToWallCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }
    @Override
    public void initialize() {
        cmd = AutoBuilder.followPath(
            new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                    driveSubsystem.getPose(),
                    driveSubsystem.getPose().plus(new Transform2d(Units.inchesToMeters(-36),0,Rotation2d.kZero))
                ), 
                new PathConstraints(2,2,Math.PI*2,Math.PI*2,12,false), 
                null, new GoalEndState(0, Rotation2d.fromDegrees(driveSubsystem.getHeading(true)))
            )
        ).until(()->driveSubsystem.getAcceleration() <= -3);
        cmd.initialize();
    }
    @Override
    public void execute() {
        cmd.execute();
    }
    @Override
    public boolean isFinished() {
        return cmd.isFinished();
    }
    @Override
    public void end(boolean interrupted) {
        cmd.end(interrupted);
    }
    
}
