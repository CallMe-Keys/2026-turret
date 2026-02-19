package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TurretLogic {
    // define motor
    public final TalonFXS neo550Motor = new TalonFXS(0);

        public TurretLogic() {
            //config and timer both need to happen only once
            TalonFXSConfiguration turretConfig = new TalonFXSConfiguration();
            turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            neo550Motor.getConfigurator().apply(turretConfig);
        }


        //limelight code
                //constants
            double kP = 0.025;
            double kI = 0.0;
            double kD = 0.001;

                //define PID
            PIDController turretPD = new PIDController(kP, kI, kD);
        
                public void track() {
            double ty = NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("ty").getDouble(0);

                //set ty to go to 0
            double motorOutput = turretPD.calculate(ty, 0);

                //apply speed with safety clamp and deadband
            if (Math.abs(ty) < 1) {
                neo550Motor.set(0);
            } else {
                neo550Motor.set(MathUtil.clamp(motorOutput, -0.1, 0.1));
            }
        }

            public void stop() {
                neo550Motor.set(0);
            }
}