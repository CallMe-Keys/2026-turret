package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;


public class ShooterLogic {

    //define motor
        public final TalonFXS krakenMotor = new TalonFXS(1);

    //defining controller
        public final XboxController controller = new XboxController(0);

    //define shooting speeds
        public double Speed = 0.625; // 0.625 for inside lab, 0.75 for field testing


    public ShooterLogic() {
        //config needs to happen only once
            TalonFXSConfiguration shooterConfig = new TalonFXSConfiguration();
            shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            krakenMotor.getConfigurator().apply(shooterConfig);
    }

    public void holdShoot() {
        if (controller.getAButton()) {
          // 0.625 for inside lab, 0.75 for field testing
          krakenMotor.set(Speed);
        } else {
          krakenMotor.set(0.0);
        }
    }

    public void pressShoot() {
        if (controller.getBButtonPressed()) {
          // 0.625 for inside lab, 0.75 for field testing
          krakenMotor.set(Speed);
        } else if (controller.getYButtonPressed()) {
          krakenMotor.set(0.0);
        }
    }
    
}
