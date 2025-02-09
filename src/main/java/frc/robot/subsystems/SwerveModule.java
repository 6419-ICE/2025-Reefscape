package frc.robot.subsystems;

import java.nio.Buffer;

import javax.swing.JList.DropLocation;

import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule implements Sendable{
    
    private final SparkFlex driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkClosedLoopController driveController;

    private final SparkFlex turnMotor;
    private final AbsoluteEncoder turnEncoder;
    private final SparkClosedLoopController turnController;

    private double chassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveID, int turningID, double angularOffset) {
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        turnMotor = new SparkFlex(turningID,MotorType.kBrushless);
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder();

        driveController = driveMotor.getClosedLoopController();
        turnController = turnMotor.getClosedLoopController();
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        turnConfig.absoluteEncoder.positionConversionFactor(Math.PI*2);
        driveConfig.encoder
            .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        

        turnConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted);

        turnConfig.closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
            .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
        driveConfig.closedLoop.pidf(
            ModuleConstants.kDrivingP,
            ModuleConstants.kDrivingI,
            ModuleConstants.kDrivingD,
            ModuleConstants.kDrivingFF
        ).outputRange(
            ModuleConstants.kDrivingMinOutput,
            ModuleConstants.kDrivingMaxOutput
        );
        // driveController.setP(ModuleConstants.kDrivingP);
        // driveController.setI(ModuleConstants.kDrivingI);
        // driveController.setD(ModuleConstants.kDrivingD);
        // driveController.setFF(ModuleConstants.kDrivingFF);
        // driveController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        // ModuleConstants.kDrivingMaxOutput);
        turnConfig.closedLoop.pidf(
            ModuleConstants.kTurningP,
            ModuleConstants.kTurningI,
            ModuleConstants.kTurningD,
            ModuleConstants.kTurningFF
        ).outputRange(
            ModuleConstants.kTurningMinOutput, 
            ModuleConstants.kTurningMaxOutput
        );
        // turnController.setP(ModuleConstants.kTurningP);
        // turnController.setI(ModuleConstants.kTurningI);
        // turnController.setD(ModuleConstants.kTurningD);
        // turnController.setFF(ModuleConstants.kTurningFF);
        // turnController.setOutputRange(ModuleConstants.kTurningMinOutput,
        // ModuleConstants.kTurningMaxOutput);
        driveConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
        turnConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);
        // driveMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        // turnMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        driveConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        turnConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        //modify these as needed
        driveConfig.inverted(true);

        turnConfig.absoluteEncoder.inverted(true);
        turnConfig.inverted(true);
        
        // driveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        // turnMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        chassisAngularOffset = angularOffset;
        desiredState.angle = new Rotation2d(turnEncoder.getPosition());
        driveEncoder.setPosition(0);
    }
    public SwerveModuleState getState() { 
        return new SwerveModuleState(driveEncoder.getVelocity(),
            new Rotation2d((turnMotor.getAbsoluteEncoder().getPosition()-chassisAngularOffset)));
    }
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d((turnMotor.getAbsoluteEncoder().getPosition() - chassisAngularOffset))
        );
    }
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
        correctedDesiredState.optimize(getPosition().angle);
        // Optimize the reference state to avoid spinning further than 90 degrees.
        // SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        //     new Rotation2d(turnEncoder.getPosition()));
    
        // Command driving and turning SPARKS MAX towards their respective setpoints.
        driveController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        
        turnController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    
        this.desiredState = correctedDesiredState;
    }
    public void driveRaw(Rotation2d angle, double power) {
        setDesiredState(new SwerveModuleState(0,angle));

        driveController.setReference(power, ControlType.kDutyCycle);

    }
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }
    public double getDriveTemp() {
        return driveMotor.getMotorTemperature();
    }
    public double getTurnTemp() {
        return turnMotor.getMotorTemperature();
    }
    public double getDriveCurrent() {
        return driveMotor.getOutputCurrent();
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Drive Motor Temp", this::getDriveTemp,null);
        builder.addDoubleProperty("Turn Motor Temp", this::getTurnTemp, null);
        builder.addDoubleProperty("Drive Motor Current", driveMotor::getOutputCurrent,null);
        builder.addDoubleProperty("Turn Motor Current", turnMotor::getOutputCurrent, null);
        SwerveModuleState state = getState();
        builder.addDoubleProperty("Drive Motor Velocity", ()->state.speedMetersPerSecond, null);
        builder.addDoubleProperty("Turn Motor Position", ()->state.angle.getDegrees(), null);
        builder.update();
    }

}
