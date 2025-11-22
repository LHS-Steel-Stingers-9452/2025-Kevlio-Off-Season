package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbIntake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(26); // Should be the right id
    private final DutyCycleOut control = new DutyCycleOut(0);

    public ClimbIntake() {
        intakeMotor.setInverted(true); // Uncomment if intake runs backwards
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    /** Spins intake inwards (pulls coral in). */
    public void intakeIn() {
        intakeMotor.setControl(control.withOutput(0.6)); // Adjust power as needed
    }

    /** Spins intake outward (outtakes coral). 
    public void intakeOut() {
        intakeMotor.setControl(control.withOutput(-0.8)); // Adjust for your needs
    } */

    /** Stops intake motor. */
    public void stop() {
        intakeMotor.setControl(control.withOutput(0));
    }
}
