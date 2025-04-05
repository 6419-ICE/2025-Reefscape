package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private DigitalInput lineBreak;
    private SparkMax motor1;
    public IntakeSubsystem() {
        lineBreak = new DigitalInput(IntakeConstants.lineBreakPort);
        motor1 = new SparkMax(IntakeConstants.intakeMotorID1, MotorType.kBrushless); {
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(false);
            config.idleMode(IdleMode.kBrake);
            motor1.configure(config,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
        }
    }
    public void setSpeed(double speed) {
        motor1.set(speed);
    }
    @Override   
    public void periodic() {
        SmartDashboard.putBoolean("Intake Has Coral", hasCoral());
        SmartDashboard.putData(lineBreak);
        SmartDashboard.updateValues();
    }
    public boolean hasCoral() {
        return !lineBreak.get();
    }
}
