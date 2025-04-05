package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPositions;

public class ElevatorSubsystem extends SubsystemBase implements Sendable {
    private SparkMax leftMotor, rightMotor;
    private SparkClosedLoopController controller;
    private DigitalInput proxSensor;
    private double goal = 0.0;
    private Timer timer = new Timer();
    public ElevatorSubsystem() {
        leftMotor = new SparkMax(ElevatorConstants.leftMotorID,MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.rightMotorID,MotorType.kBrushless);
        proxSensor = new DigitalInput(ElevatorConstants.switchPort);
        //config left motor
        {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
            //config.absoluteEncoder.countsPerRevolution(8192);
            config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            //config.alternateEncoder.positionConversionFactor(0.75*Math.PI);
            config.inverted(true);
            leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            controller = leftMotor.getClosedLoopController();
        }
        //config right motor
        {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
            config.follow(leftMotor,false);
            rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }
    public void setPosition(double pos) {
        if (pos == ElevatorPositions.inside.getValue() && goal == 0.0) return;
        goal = pos;
        controller.setReference(goal/(0.75*Math.PI/6.0), ControlType.kPosition);
    }
    /**
     * DO NOT USE THIS UNLESS YOU ABSOLUTELY, 100% KNOW WHAT YOU ARE DOING
     * @param power
     */
    public void setPower(double power) {
        controller.setReference(power,ControlType.kDutyCycle);
    }
    public double getPosition() {
        //System.out.println(((leftMotor.getEncoder().getPosition()+rightMotor.getEncoder().getPosition())/2.0) * -(0.75*Math.PI/6.0));
        return ((leftMotor.getEncoder().getPosition()+rightMotor.getEncoder().getPosition())/2.0) * (0.75*Math.PI/6.0);
    }
    public boolean atPosition() {
        return Math.abs(goal-getPosition()) <= ElevatorConstants.tolerance;
    }
    @Override
    public void periodic() {
        if (!MathUtil.isNear(0, getPosition(), 3)) return;
        if (!proxSensor.get()) {
            //if (goal == ElevatorPositions.inside.getValue()) setPosition(0.0);
            leftMotor.getEncoder().setPosition(0);
            rightMotor.getEncoder().setPosition(0);
        }
    }  
    public boolean atZero() {
        return !proxSensor.get();
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Average Position",()->(leftMotor.getEncoder().getPosition()+rightMotor.getEncoder().getPosition())/2.0 , null);
        builder.addDoubleProperty("Pos Conversion Factor", ()->-(0.75*Math.PI/6.0), null);
        builder.addDoubleProperty("Goal", ()->goal, null);
        builder.addDoubleProperty("Position", this::getPosition, null);
        builder.addDoubleProperty("Left Current", leftMotor::getOutputCurrent, null);
        builder.addDoubleProperty("Right Current", rightMotor::getOutputCurrent, null);
        builder.addDoubleProperty("Left Voltage", leftMotor::getBusVoltage, null);
        builder.addDoubleProperty("Right Voltage", rightMotor::getBusVoltage, null);
        builder.addDoubleProperty("Left Output", leftMotor::getAppliedOutput, null);
        builder.addDoubleProperty("Right Output", rightMotor::getAppliedOutput, null);
        builder.addDoubleProperty("Left Temp", leftMotor::getMotorTemperature, null);
        builder.addDoubleProperty("Right Temp", rightMotor::getMotorTemperature, null);
        builder.addDoubleProperty("Left Position", leftMotor.getEncoder()::getPosition, null);
        builder.addDoubleProperty("Right Position", rightMotor.getEncoder()::getPosition, null);
        builder.addBooleanProperty("Elevator down", this::atZero, null);
    }
}
