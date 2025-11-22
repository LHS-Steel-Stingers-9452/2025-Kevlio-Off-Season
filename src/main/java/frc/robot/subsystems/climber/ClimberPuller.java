package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberPuller extends SubsystemBase {


    private final TalonFX  climberKraken = new TalonFX(10);
    
    public ClimberPuller() {

        var motorOutputConfig =
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        var currentLimitConfig =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(100)
                .withSupplyCurrentLimit(50)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);

        var feedbackConfig =
            new FeedbackConfigs()
                .withSensorToMechanismRatio(125);

        var talonFXConfig = 
            new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfig)
            .withMotorOutput(motorOutputConfig)
            .withFeedback(feedbackConfig);
        
       
            
        
        climberKraken.getConfigurator().apply(talonFXConfig);

        setDefaultCommand(stopClimber());
    }
   

    public Command stopClimber() {
        return runOnce(() -> {
            climberKraken.set(0);
        });
       
    }

    public Command runClimber(double speed){
        return run(() -> {
            climberKraken.set(speed);
        });
    }

    @Logged
    public double climberPosition() {
        return climberKraken.getPosition().getValueAsDouble();
    }


    public void defaultClimberEncoder(){
        climberKraken.setPosition(0);
    }

}