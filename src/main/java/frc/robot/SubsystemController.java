package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
    /** ---------------------------------BUTTON BOX CONTROLS---------------------------------
    * 
    *<p>1 (Hold): Go to L2</p>
    *<p>2 (Hold): Go to L3</p>
    *<p>3 (Hold): Go to L4</p>
    *<p>4 (Hold?): Intake Note (Intake + Store in Outtake)</p>
    *<p>5 (Hold): Outtake Note</p>
    *<p>6 (Press/Hold?): Hang </p>
    *<p>7 (Hold): Use Algae Remover</p>
    * 
    */ 
public class SubsystemController extends GenericHID implements Sendable {
    private Trigger intake, outtake, hang, elevator, algaeRemover;
    public SubsystemController(int port) {
        super(port);
        elevator = new JoystickButton(this, 1)
            .or(new JoystickButton(this, 2))
            .or(new JoystickButton(this, 3));
        intake = new JoystickButton(this, 4);
        outtake = new JoystickButton(this, 5);
        hang = new JoystickButton(this, 6);
        algaeRemover = new JoystickButton(this, 7);
    }
    public int getSelectedLevel() {
        //iterate over the elevator buttons, starting at "L4" and going down. 
        for (int i = 3; i >= 0; i--) {
            if (getRawButton(i)) return i+1; //returns the button ID + 1
        }
        return 0;
    }
    public Trigger getIntake() {
        return intake;
    }
    public Trigger getOuttake() {
        return outtake;
    }
    public Trigger getHang() {
        return hang;
    }
    public Trigger getAlgaeRemover() {
        return algaeRemover;
    }
    public Trigger getElevatorButtons() {
        return elevator;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addIntegerProperty("Selected Elevator Level", this::getSelectedLevel, null);
    }
}
