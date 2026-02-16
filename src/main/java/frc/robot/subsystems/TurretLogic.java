package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;

public class TurretLogic {
    // define motor and timer
    public final TalonFXS neo550Motor = new TalonFXS(0); 
    private final Timer timer = new Timer();

    public double LeftSpeed = 0.1;
    public double RightSpeed = -0.1;

    // constants, must be outside loop
    private double lastError = 0;
    private double lastTime = 0;

    private double kP = 0.05; // note, fine tune these constants
    private double kD = 0.00;

    public TurretLogic() {
        //config and timer both need to happen only once
        TalonFXSConfiguration turretConfig = new TalonFXSConfiguration();
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        neo550Motor.getConfigurator().apply(turretConfig);
        
        timer.start();
    }


    public void calculateAndMove(double error) {
        double currentTime = timer.get();
        double deltaTime = currentTime - lastTime;
        
        double pTerm = error * kP;
        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm = (error - lastError) / deltaTime * kD;
        }

        // Apply power
        double power = pTerm + dTerm; 
        
        // Use a small deadband to stop jitter
        if (Math.abs(error) < 0.2) {
            neo550Motor.set(0);
        } else {
            // Clamp and set
            double clampedPower = Math.max(-0.2, Math.min(0.2, power));
            neo550Motor.set(clampedPower);
        }

        // Save values for next loop
        lastError = error;
        lastTime = currentTime;
    }

    public void stop() {
        neo550Motor.set(0);
    }
}