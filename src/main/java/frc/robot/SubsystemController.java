package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SubsystemController extends GenericHID {
    private Trigger intake, outtake, hang, elevator, algaeRemover;
    public SubsystemController(int port) {
        super(port);
        elevator = new JoystickButton(this, 0)
            .or(new JoystickButton(this, 1))
            .or(new JoystickButton(this, 2))
            .or(new JoystickButton(this, 3));
        intake = new JoystickButton(this, 4);
        outtake = new JoystickButton(this, 5);
        hang = new JoystickButton(this, 6);
        algaeRemover = new JoystickButton(this, 7);
    }
    public int getSelectedLevel() {
        for (int i = 3; i >= 0; i--) {
            if (getRawButtonReleased(i)) return i+1;
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
}
