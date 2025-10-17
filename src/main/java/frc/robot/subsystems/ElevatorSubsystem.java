package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPositions;

public class ElevatorSubsystem extends SubsystemBase implements Sendable {
    private SparkMax leftMotor, rightMotor;
    private SparkClosedLoopController controller;
    //private DigitalInput proxSensor;
    private double goal = ElevatorConstants.minHeight;
    private LaserCan distanceSensor;
    private PIDController pidController;
    private double prevPosition = -1;
    private Timer prevPositionTimer = new Timer();
    //blocks elevator movement when true, must be set false by driver 1 input
    private ElevatorFaultType elevatorFault = ElevatorFaultType.NONE;
    public enum ElevatorFaultType {
        NONE,
        INVALID_POSITION_CHANGE,
        POSITION_STUCK,
        POSITION_OUT_OF_BOUNDS
    }
    public ElevatorSubsystem() {
        leftMotor = new SparkMax(ElevatorConstants.leftMotorID,MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.rightMotorID,MotorType.kBrushless);
        //proxSensor = new DigitalInput(ElevatorConstants.switchPort);
        //config left motor
        {
            SparkMaxConfig config = new SparkMaxConfig();
            //config.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
            //config.absoluteEncoder.countsPerRevolution(8192);
            //config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            //config.alternateEncoder.positionConversionFactor(0.75*Math.PI);
            config.inverted(true);
            config.smartCurrentLimit(30);
            leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            controller = leftMotor.getClosedLoopController();
        }
        //config right motor
        {
            SparkMaxConfig config = new SparkMaxConfig();
            //config.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
            config.follow(leftMotor,false);
            rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        //leftMotor.getEncoder().setPosition(0);
        //rightMotor.getEncoder().setPosition(0);
        distanceSensor = new LaserCan(Constants.ElevatorConstants.laserCANid);
        pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        pidController.setTolerance(ElevatorConstants.tolerance);
        pidController.setSetpoint(goal);
        SmartDashboard.putData("Elevator PID", pidController);
        prevPositionTimer.stop();
    }
    public void setPosition(double pos) {
        //if (pos == ElevatorPositions.inside.getValue() && goal == 0.0) return;
        goal = pos;
        pidController.setSetpoint(pos);
        //controller.setReference(goal/(0.75*Math.PI/6.0), ControlType.kPosition);
    }
    /**
     * DO NOT USE THIS UNLESS YOU ABSOLUTELY, 100% KNOW WHAT YOU ARE DOING
     * @param power
     */
    public void setPower(double power) {
    
        controller.setReference(power,ControlType.kDutyCycle);
    }
    public double getPosition() {

        if (distanceSensor.getMeasurement() == null) {
            DriverStation.reportWarning("LaserCAN ID 18 not responding: No measurement", new StackTraceElement[0]);
            return -1;
        }
        double distInches = Units.metersToInches(distanceSensor.getMeasurement().distance_mm/1000.0) + Constants.ElevatorConstants.measurementOffset;
        if (distInches > ElevatorConstants.maxHeight+ElevatorConstants.tolerance || distInches < ElevatorConstants.minHeight-ElevatorConstants.tolerance) {
            DriverStation.reportWarning("LaserCAN is reporting an out of bounds value ("+distInches+")", new StackTraceElement[0]);
            return -1;
        }
        //System.out.println(((leftMotor.getEncoder().getPosition()+rightMotor.getEncoder().getPosition())/2.0) * -(0.75*Math.PI/6.0));
        return Units.metersToInches(distanceSensor.getMeasurement().distance_mm/1000.0) + Constants.ElevatorConstants.measurementOffset;
        //return ((leftMotor.getEncoder().getPosition()+rightMotor.getEncoder().getPosition())/2.0) * (0.75*Math.PI/6.0);
    }
    public boolean atPosition() {
        return MathUtil.isNear(goal, getPosition(), ElevatorConstants.tolerance);//Math.abs(goal-getPosition()) <= ElevatorConstants.tolerance;
    }
    @Override
    public void periodic() {
        if (elevatorFault != ElevatorFaultType.NONE) {
            DriverStation.reportWarning("Elevator fault present ("+elevatorFault+"), press \"Circle\" to clear", new StackTraceElement[0]);
            setPower(0.0);
            prevPosition = getPosition();
            LEDSubsystem.setElevatorPattern(LEDSubsystem.ELEVATOR_ERR);
            return;
        }
        LEDSubsystem.setElevatorPattern(LEDSubsystem.ELEVATOR_STANDARD);
       

        
        double pos = getPosition();
        //blockage handling
        //check for no movement
        if ((MathUtil.isNear(pos, prevPosition, ElevatorConstants.tolerance) && !atPosition()) && DriverStation.isEnabled()) {
            if (prevPositionTimer.isRunning()) {
                if (prevPositionTimer.hasElapsed(0.5)) {
                    pauseElevator(ElevatorFaultType.POSITION_STUCK);
                    prevPosition = pos;
                    prevPositionTimer.stop();
                    return;
                }
            } else {
                prevPositionTimer.restart();
            }
        } else {
            prevPositionTimer.stop();
        }
        //check for rapid change in position
        if (prevPosition != -1) {
            if (Math.abs(prevPosition - pos) > 15) {
                pauseElevator(ElevatorFaultType.INVALID_POSITION_CHANGE);
                prevPosition = pos;
                return;
            }
        }

        //check for out of bounds positions
        if (pos == -1) {
            pauseElevator(ElevatorFaultType.POSITION_OUT_OF_BOUNDS);
            prevPosition = pos;
            return; 
        } 
        //PID calculations
        setPower(atZero() && atPosition() ? 0.0 : pidController.calculate(getPosition())); //dont move if elevator is at zero, to avoid slowly burning out motors
        prevPosition = pos;
        //if (!MathUtil.isNear(0, getPosition(), 3)) return;
        // if (!proxSensor.get()) {
        //     //if (goal == ElevatorPositions.inside.getValue()) setPosition(0.0);
        //     leftMotor.getEncoder().setPosition(0);
        //     rightMotor.getEncoder().setPosition(0);
        // }
        
    }  
    private void pauseElevator(ElevatorFaultType fault) {
        //DriverStation.reportWarning("Elevator paused, press \"Circle\" to unpause", new StackTraceElement[0]);
        elevatorFault = fault;
        setPower(0.0);
    }
    public boolean atZero() {
        return MathUtil.isNear(ElevatorConstants.minHeight, getPosition(), ElevatorConstants.tolerance); //!proxSensor.get();
    }
    public void clearManualPause() {
        elevatorFault = ElevatorFaultType.NONE;
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
        builder.addBooleanProperty("Elevator at Position", this::atPosition, null);
        builder.addDoubleProperty("PID Error", pidController::getError, null);
        builder.addBooleanProperty("Elevator Paused", ()->elevatorFault != ElevatorFaultType.NONE, null);
    }
}
