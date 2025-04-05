// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorAndOuttakePositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.Constants.IntakeStates;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OuttakeAngles;
import frc.robot.Constants.OuttakeStates;
import frc.robot.commands.DriveToWallCommand;
import frc.robot.commands.ElevatorStressTest;
import frc.robot.commands.FullIntakeCommand;
import frc.robot.commands.FullOuttakeCommand;
import frc.robot.commands.MoveElevatorAndOuttakeCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.OrientToAprilTagCommand;
import frc.robot.commands.PowerOuttakeRotationCommand;
import frc.robot.commands.RotateOuttakeCommand;
import frc.robot.commands.RunFlipperCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunOuttakeCommand;
import frc.robot.commands.TestOuttakeCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem.Side;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final LimelightSubsystem leftLimelightSubsystem, rightLimelightSubsystem;
  private final LEDSubsystem ledSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driveController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  private final SubsystemController buttonBox = 
      new SubsystemController(1);

  private int elevatorLevel = 0;
  private SendableChooser<Command> commandChooser;
  public static Field2d field;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //subsystem definition
    elevatorSubsystem = new ElevatorSubsystem();
    outtakeSubsystem = new OuttakeSubsystem();
    flipperSubsystem = new FlipperSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    leftLimelightSubsystem = new LimelightSubsystem(Side.left);
    rightLimelightSubsystem = new LimelightSubsystem(Side.right);
    ledSubsystem = new LEDSubsystem();
    driveSubsystem = new DriveSubsystem();
    defineNamedCommands();
    driveSubsystem.configAutoBuilder();
    field = new Field2d();
    commandChooser = AutoBuilder.buildAutoChooser();
    
    //defineNamedCommands();
    commandChooser.addOption("Test Outtake", new TestOuttakeCommand(outtakeSubsystem));
    commandChooser.addOption("Spin Outtake", new PowerOuttakeRotationCommand(-0.2,outtakeSubsystem));
    commandChooser.addOption("Test Elevator", new ElevatorStressTest(elevatorSubsystem).repeatedly().withTimeout(Units.Minutes.of(5)));
    commandChooser.addOption("Zero Oueftttake", new RotateOuttakeCommand(OuttakeAngles.inside, outtakeSubsystem));
    commandChooser.addOption("Left L2", new PathPlannerAuto("Right L2",true));
    SmartDashboard.putData(commandChooser);
    SmartDashboard.putData("xTranslationController", Constants.LimelightConstants.xTranslationController);
    SmartDashboard.putData("zTranslationController", Constants.LimelightConstants.zTranslationController);
    SmartDashboard.putData("rotationController",Constants.LimelightConstants.rotationController); 
    SmartDashboard.putData("Elevator",elevatorSubsystem);
    SmartDashboard.putData("Button Box",buttonBox);
    SmartDashboard.putData("Outtake",outtakeSubsystem);
    SmartDashboard.putData("Left Limelight",leftLimelightSubsystem);
    SmartDashboard.putData("Right Limelight", rightLimelightSubsystem);
    SmartDashboard.putData(field);
    // Configure the trigger bindings
    //configureBindings();
    configureBindings();

    Logger.init();
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
      driveController.cross().whileTrue(new RunCommand(driveSubsystem::setX, driveSubsystem));
      //Drive to Left Side Reef
      driveController.L1().and(driveController.R1().negate()).whileTrue(
        new OrientToAprilTagCommand(rightLimelightSubsystem, driveSubsystem)
      );
      //Drive to Right Side Reef
      driveController.R1().and(driveController.L1().negate()).whileTrue(
        new OrientToAprilTagCommand(leftLimelightSubsystem,driveSubsystem)
      );
      //Control Elevator
      buttonBox.getElevatorButtons()
        .whileTrue(new MoveElevatorAndOuttakeCommand(()->ElevatorAndOuttakePositions.getLevel(buttonBox.getSelectedLevel()), elevatorSubsystem, outtakeSubsystem))
        .whileFalse(Commands.sequence(
          new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.intake, elevatorSubsystem, outtakeSubsystem),
          new ZeroElevatorCommand(elevatorSubsystem)
        ));
        // buttonBox.getElevatorButtons().toggleOnTrue(
        //   Commands.parallel(
        //     new MoveElevatorAndOuttakeCommand(()->ElevatorAndOuttakePositions.getLevel(buttonBox.getSelectedLevel()), elevatorSubsystem, outtakeSubsystem),
        //     new InstantCommand().repeatedly()
        //   )
        //   .handleInterrupt(()-> Commands.sequence(
        //     new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.intake, elevatorSubsystem, outtakeSubsystem),
        //     new ZeroElevatorCommand(elevatorSubsystem)).schedule()
        //   )
        // );
      //Intake Coral
      buttonBox.getIntake().whileTrue(new FullIntakeCommand(intakeSubsystem, elevatorSubsystem, outtakeSubsystem));
      //Outtake Coral
      buttonBox.getOuttake().whileTrue(
        new ConditionalCommand(
          new RunOuttakeCommand(OuttakeStates.intake, outtakeSubsystem),
          new RunOuttakeCommand(OuttakeStates.outtake, outtakeSubsystem),
          ()->buttonBox.getSelectedLevel() == 4
        )
        );
      buttonBox.getHang().whileTrue(new RunOuttakeCommand(OuttakeStates.reverse, outtakeSubsystem));
      //Remove Algae
  }
  private void configureBindingsElevatorStringing() {
    buttonBox.getElevatorButtons().whileTrue(new ConditionalCommand(
      new FunctionalCommand(()->elevatorSubsystem.setPower(0.05), ()->{}, (bool)->elevatorSubsystem.setPower(0), ()->false, elevatorSubsystem), 
      new FunctionalCommand(()->elevatorSubsystem.setPower(-0.05), ()->{}, (bool)->elevatorSubsystem.setPower(0), ()->false, elevatorSubsystem), 
      ()->buttonBox.getSelectedLevel() == 2
    ));
  }
  private void defineNamedCommands() {
    NamedCommands.registerCommand("Elevator L2",new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.L2, elevatorSubsystem, outtakeSubsystem));
    NamedCommands.registerCommand("Score Note", Commands.parallel(
          new RunIntakeCommand(IntakeStates.intake, intakeSubsystem),
          new RunOuttakeCommand(OuttakeStates.outtake, outtakeSubsystem)
        ));
    NamedCommands.registerCommand("Zero Elevator", new ZeroElevatorCommand(elevatorSubsystem));
    NamedCommands.registerCommands(
      List.of(
        Pair.of(
          "Score L2 Left",
           Commands.sequence(
            Commands.parallel(
              new OrientToAprilTagCommand(rightLimelightSubsystem, driveSubsystem).withTimeout(2),
              new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.L2, elevatorSubsystem, outtakeSubsystem)
            ),
            new RunOuttakeCommand(OuttakeStates.outtake, outtakeSubsystem).withTimeout(0.9)
           )
        ),
        Pair.of(
          "Score L2 Right",
           Commands.sequence(
            Commands.parallel(
              new OrientToAprilTagCommand(leftLimelightSubsystem, driveSubsystem).withTimeout(2),
              new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.L2, elevatorSubsystem, outtakeSubsystem)
            ),
            new RunOuttakeCommand(OuttakeStates.outtake, outtakeSubsystem).withTimeout(0.9
            )
           )
        ),
        Pair.of(
          "Score L4 Left",
           Commands.sequence(
            Commands.parallel(
              new OrientToAprilTagCommand(rightLimelightSubsystem, driveSubsystem).withTimeout(5),
              new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.L4, elevatorSubsystem, outtakeSubsystem)
            ),
            new RunOuttakeCommand(OuttakeStates.outtake, outtakeSubsystem).withTimeout(0.9)
           )
        ),
        Pair.of(
          "Score L4 Right",
           Commands.sequence(
            Commands.parallel(
              new OrientToAprilTagCommand(leftLimelightSubsystem, driveSubsystem).withTimeout(5),
              new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.L4, elevatorSubsystem, outtakeSubsystem)
            ),
            new RunOuttakeCommand(OuttakeStates.outtake, outtakeSubsystem).withTimeout(0.9
            )
           )
        ),
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
          "Intake note", 
          new FullIntakeCommand(intakeSubsystem, elevatorSubsystem, outtakeSubsystem)
        ),
        Pair.of("Score Note", Commands.parallel(
          new RunIntakeCommand(IntakeStates.intake, intakeSubsystem),
          new RunOuttakeCommand(OuttakeStates.outtake, outtakeSubsystem)
        )),
        Pair.of("Zero Elevator",
          Commands.sequence(
            new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.intake, elevatorSubsystem, outtakeSubsystem),
            new ZeroElevatorCommand(elevatorSubsystem)
          )
        ),
        Pair.of("Rotate Outtake Manual",new PowerOuttakeRotationCommand(-0.2, outtakeSubsystem).withTimeout(3)),
        Pair.of("Align Left", new OrientToAprilTagCommand(rightLimelightSubsystem, driveSubsystem)),
        Pair.of("Align Right", new OrientToAprilTagCommand(leftLimelightSubsystem, driveSubsystem))
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
