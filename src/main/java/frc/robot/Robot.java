// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
//import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  // defining motor (id must match phoenix tuner)
    private final TalonFXS neo550TurretMotor = new TalonFXS(0); 
    private final TalonFXS krakenShooterMotor = new TalonFXS(1); 

    // defining base rpm -1 to 1 decimal
    private final DutyCycleOut percentOutput = new DutyCycleOut(0);

    //defining controller
    private final XboxController controller = new XboxController(0);

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    //this makes config for kraken and 550
      TalonFXSConfiguration shooterConfig = new TalonFXSConfiguration();
      TalonFXSConfiguration turretConfig = new TalonFXSConfiguration();
    
    //applys brake as neutral to both motors
      shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      /* EXPERIMENTAL limit switch 
    //enable physical limit switch
      turretConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
      turretConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
      turretConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
      turretConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    //sets forward soft limit to 100 rotation for turret
      turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100;
      turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -100; */

    //applys configs to motors
      krakenShooterMotor.getConfigurator().apply(shooterConfig);
      neo550TurretMotor.getConfigurator().apply(turretConfig);


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // if auto is running during teleop init, cancel it
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
//turret teleop controls CHANGE FOR OPERATOR

        // *EXPERIMENTAL* left joystick X axis = turret spinning.
          //if (Math.abs(controller.getLeftX()) < 0.1) {
        // because of config, it should brake when value 0 is called.
          //neo550TurretMotor.set(0); 
        //} else {
        //neo550TurretMotor.set(controller.getLeftX());
        //} 

                // press y to run turret = 5% neo 550 turret.
         if (controller.getYButton()) {
          // between 5 to 10 percent speed for turret
          neo550TurretMotor.setControl(percentOutput.withOutput(0.05));
        } else {
          neo550TurretMotor.setControl(percentOutput.withOutput(0.0));
        }

                // press x to run turret = reverse 5% neo 550 turret.
         if (controller.getXButton()) {
          // between 5 to 10 percent reverse speed for turret
          neo550TurretMotor.setControl(percentOutput.withOutput(-0.05));
        } else {
          neo550TurretMotor.setControl(percentOutput.withOutput(0.0));
        }


        // press x to run shooter = half speed forward kraken.
         if (controller.getAButton()) {
          // 0.625 for inside lab, 0.75 for field testing
          krakenShooterMotor.setControl(percentOutput.withOutput(0.625));
        } else {
          krakenShooterMotor.setControl(percentOutput.withOutput(0.0));
        }
    }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
