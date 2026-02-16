package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterLogic {

    //define motor
        public final TalonFXS krakenMotor = new TalonFXS(1);

    //define turret speeds
        public double Speed = 0.625; // 0.625 for inside lab, 0.75 for field testing


    public ShooterLogic() {
        //config needs to happen only once
            TalonFXSConfiguration shooterConfig = new TalonFXSConfiguration();
            shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            krakenMotor.getConfigurator().apply(shooterConfig);
    }
}
