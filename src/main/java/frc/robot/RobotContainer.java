// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorAndOuttakePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OuttakeStates;
import frc.robot.commands.DriveToWallCommand;
import frc.robot.commands.FullIntakeCommand;
import frc.robot.commands.FullOuttakeCommand;
import frc.robot.commands.MoveElevatorAndOuttakeCommand;
import frc.robot.commands.OrientToAprilTag;
import frc.robot.commands.RunFlipperCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunOuttakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //subsystem declaration 
  private final DriveSubsystem driveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final OuttakeSubsystem outtakeSubsystem;
  private final FlipperSubsystem flipperSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final LimelightSubsystem limelightSubsystem;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driveController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  private final SubsystemController buttonBox = 
      new SubsystemController(1);

  private int elevatorLevel = 0;

  private SendableChooser<Command> commandChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //subsystem definition
    driveSubsystem = new DriveSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    outtakeSubsystem = new OuttakeSubsystem();
    flipperSubsystem = new FlipperSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    limelightSubsystem = new LimelightSubsystem();
    commandChooser = AutoBuilder.buildAutoChooser();
    defineNamedCommands();
    SmartDashboard.putData(commandChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
      //default driving 
      driveSubsystem.setDefaultCommand(
        new RunCommand(()->{
          driveSubsystem.drive(
            MathUtil.applyDeadband(-driveController.getLeftY(), 0.06),
            MathUtil.applyDeadband(-driveController.getLeftX(), 0.06),
            MathUtil.applyDeadband(-driveController.getRightX(), 0.06),
            true);
        }, 
        driveSubsystem
      ).handleInterrupt(()->driveSubsystem.drive(0, 0, 0, false)));

      driveController.L1().or(driveController.R1())
        .whileTrue(new OrientToAprilTag(buttonBox.getSelectedLevel(), driveController.L1().getAsBoolean(), limelightSubsystem, driveSubsystem).andThen(new DriveToWallCommand(driveSubsystem)));
      
      elevatorSubsystem.setDefaultCommand(
        new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.getLevel(buttonBox.getSelectedLevel()), elevatorSubsystem, outtakeSubsystem)
      );

      buttonBox.getIntake().onTrue(new FullIntakeCommand(intakeSubsystem, elevatorSubsystem, outtakeSubsystem));

      buttonBox.getOuttake().whileTrue(new RunOuttakeCommand(OuttakeStates.outtake, outtakeSubsystem));
        
      buttonBox.getAlgaeRemover().whileTrue(new RunFlipperCommand(flipperSubsystem));
  }
  private void defineNamedCommands() {
    NamedCommands.registerCommands(
      List.of(
        Pair.of(
          "Elevator Inside", 
          new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.inside, elevatorSubsystem, outtakeSubsystem)
        ),
        Pair.of(
          "Elevator L1", 
          new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.L1, elevatorSubsystem, outtakeSubsystem)
        ),
        Pair.of(
          "Elevator L2", 
          new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.L2, elevatorSubsystem, outtakeSubsystem)
        ),
        Pair.of(
          "Elevator L3", 
          new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.L3, elevatorSubsystem, outtakeSubsystem)
        ),
        Pair.of(
          "Elevator L4", 
          new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.L4, elevatorSubsystem, outtakeSubsystem)
        ),
        Pair.of(
            "Score L1 left",
            new FullOuttakeCommand(1, true, outtakeSubsystem, elevatorSubsystem, limelightSubsystem, driveSubsystem)
        ),
        Pair.of(
            "Score L1 right",
            new FullOuttakeCommand(1, false, outtakeSubsystem, elevatorSubsystem, limelightSubsystem, driveSubsystem)
        ),
        Pair.of(
            "Score L2 left",
            new FullOuttakeCommand(2, true, outtakeSubsystem, elevatorSubsystem, limelightSubsystem, driveSubsystem)
        ),
        Pair.of(
            "Score L2 right",
            new FullOuttakeCommand(1, false, outtakeSubsystem, elevatorSubsystem, limelightSubsystem, driveSubsystem)
        ),
        Pair.of(
            "Score L3 left",
            new FullOuttakeCommand(3, true, outtakeSubsystem, elevatorSubsystem, limelightSubsystem, driveSubsystem)
        ),
        Pair.of(
            "Score L3 right",
            new FullOuttakeCommand(3, false, outtakeSubsystem, elevatorSubsystem, limelightSubsystem, driveSubsystem)
        ),
        Pair.of(
            "Score L4 left",
            new FullOuttakeCommand(4, true, outtakeSubsystem, elevatorSubsystem, limelightSubsystem, driveSubsystem)
        ),
        Pair.of(
            "Score L4 right",
            new FullOuttakeCommand(4, false, outtakeSubsystem, elevatorSubsystem, limelightSubsystem, driveSubsystem)
        ),
        Pair.of(
          "Intake note", 
          new FullIntakeCommand(intakeSubsystem, elevatorSubsystem, outtakeSubsystem)
          )
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return commandChooser.getSelected();
  }
}
