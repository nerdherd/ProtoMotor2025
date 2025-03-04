// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.DoubleBinaryOperator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.util.preferences.PrefBool;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public HashMap<Integer, TalonFX> motors = new HashMap<Integer, TalonFX>();
  public HashMap<Integer, Double> relSpeeds = new HashMap<Integer, Double>();
  public HashMap<Integer, VelocityVoltage> velocityRequests = new HashMap<Integer, VelocityVoltage>();
  public HashMap<Integer, PrefBool> enabled = new HashMap<Integer, PrefBool>();
  public HashMap<Integer, PIDController> pidControllers = new HashMap<Integer, PIDController>();
  public HashMap<Integer, Boolean> usePID = new HashMap<Integer, Boolean>();
  public HashMap<Integer, Double> pidPositions = new HashMap<Integer, Double>();
  public ShuffleboardTab tab = Shuffleboard.getTab("MotorTesting");
  public CommandPS4Controller controller = new CommandPS4Controller(0);
  public double rpm = 0.0;
  public double max_rpm = 0.1;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Constants.kRPM.loadPreferences();
    Constants.kP.loadPreferences();
    Constants.kI.loadPreferences();
    Constants.kD.loadPreferences();
    Constants.kV.loadPreferences();
    for (Constants.MotorIDs id : Constants.MotorIDs.values()) {
      int i = id.id;
      TalonFX motor = new TalonFX(i);
      VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
      motor.setControl(velocityRequest);
      motors.put(i, motor);

      relSpeeds.put(i,id.relspeed);

      velocityRequests.put(i, velocityRequest);
      PrefBool b = new PrefBool("Motor" + Integer.toString(i), false);
      enabled.put(i, b);
      b.loadPreferences();
      
      TalonFXConfigurator configurator = motor.getConfigurator();
      TalonFXConfiguration config = new TalonFXConfiguration();
      configurator.refresh(config);
      PIDController pidController = new PIDController(config.Slot0.kP, config.Slot0.kI, config.Slot0.kD);
      pidControllers.put(i, pidController);
      pidPositions.put(i, 0.0);
      usePID.put(i, false);
      tab.addDouble("Motor" + Integer.toString(i) + " Velocity", () -> motor.getVelocity().getValueAsDouble());
      tab.addDouble("Motor" + Integer.toString(i) + " Position", () -> motor.getPosition().getValueAsDouble());
      tab.addBoolean("Motor" + Integer.toString(i) + " Use PID", () -> usePID.get(i));
    }
    SmartDashboard.putData("Stop All", stopAllMotorsCommand());
    SmartDashboard.putData("Set RPM of Enabled", setRPMsOfEnabledCommand());
    SmartDashboard.putData("Set Position of Enabled", setPositionsOfEnabledCommand());
    SmartDashboard.putData("Set PID of Enabled", setPIDsOfEnabledCommand());

    controller.cross().onTrue(setRPMsOfEnabledCommand())
                      .onFalse(stopAllMotorsCommand());
    controller.R2().onTrue(Commands.runOnce(() -> rpm = max_rpm))
                      .onFalse(Commands.runOnce(() -> rpm = 0.0));
    controller.L2().onTrue(Commands.runOnce(() -> rpm = -max_rpm))
                      .onFalse(Commands.runOnce(() -> rpm = 0.0));
  }

  void stopAllMotors() {
    for (Constants.MotorIDs id : Constants.MotorIDs.values()) {
      velocityRequests.get(id.id).Velocity = 0;
      motors.get(id.id).setVoltage(0);
      usePID.put(id.id, false);
    }
  }

  void setRPMsOfEnabled() {
    Constants.kRPM.loadPreferences();
    // double RPM = 2.0;
    for (Constants.MotorIDs id : Constants.MotorIDs.values()) {
      int i = id.id;
      enabled.get(i).loadPreferences();
      if (true || enabled.get(id.id).get()) {
        usePID.put(i, false);

        // if(id.id == 61) {
        //   motors.get(id.id).set(rpm * 2); // TODO uh oh
        // }
        // if(id.id == 62) {
        //   motors.get(id.id).set(rpm); // TODO uh oh
        // }

        motors.get(i).set(rpm * relSpeeds.get(i));

        // DriverStation.reportWarning("RPM: " + Double.toString(Constants.kRPM.get()), false);
        pidControllers.get(i).reset();
        // motors.get(id.id).setControl(velocityRequests.get(id.id));
        // if (Math.abs(Constants.kRPM.get()) < 0.01) {
        //   motors.get(id.id).setVoltage(0);
        // }
      }
    }
  }

  void setPositionsOfEnabled() {
    Constants.kPosition.loadPreferences();
    for (Constants.MotorIDs id : Constants.MotorIDs.values()) {
      enabled.get(id.id).loadPreferences();
      if (enabled.get(id.id).get()) {
        usePID.put(id.id, true);
        pidPositions.put(id.id, Constants.kPosition.get());
      }
    }
  }

  void setPIDsOfEnabled() {
    Constants.kP.loadPreferences();
    Constants.kI.loadPreferences();
    Constants.kD.loadPreferences();
    Constants.kV.loadPreferences();
    for (Constants.MotorIDs id : Constants.MotorIDs.values()) {
      enabled.get(id.id).loadPreferences();
      if (enabled.get(id.id).get()) {
        setMotorPID(id.id, Constants.kP.get(), Constants.kI.get(), Constants.kD.get(), Constants.kV.get());
      }
    }
  }

  Command stopAllMotorsCommand() {
    return Commands.runOnce(() -> stopAllMotors());
  }

  Command setRPMsOfEnabledCommand() {
    return Commands.runOnce(() -> setRPMsOfEnabled());
  }

  Command setPositionsOfEnabledCommand() {
    return Commands.runOnce(() -> setPositionsOfEnabled());
  }

  Command setPIDsOfEnabledCommand() {
    return Commands.runOnce(() -> setPIDsOfEnabled());
  }

  void setMotorPID(int id, double perbosity, double ierbosity, double derbosity, double verbosity) {
    TalonFX mootor = motors.get(id);
    TalonFXConfigurator configurator = mootor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    configurator.refresh(config);
    config.Slot0.kP = perbosity;
    config.Slot0.kI = ierbosity;
    config.Slot0.kD = derbosity;
    config.Slot0.kV = verbosity;
    PIDController controller = pidControllers.get(id);
    controller.setP(perbosity);
    controller.setI(ierbosity);
    controller.setD(derbosity);
    StatusCode status = configurator.apply(config);
    if (!status.isOK()){
      DriverStation.reportError("Could not apply shooter configs, error code:"+ status.toString(), new Error().getStackTrace());
    }
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
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    for (Constants.MotorIDs id : Constants.MotorIDs.values()) {
      if (usePID.get(id.id)) {
        double val = pidControllers.get(id.id).calculate(motors.get(id.id).getPosition().getValueAsDouble(), pidPositions.get(id.id));
        if (val == val) motors.get(id.id).set(val); // checks if val is a number (nan != nan)
        // motors.get(id.id).setControl(velocityRequests.get(id.id));
        if (pidControllers.get(id.id).atSetpoint() || (Math.abs(motors.get(id.id).getPosition().getValueAsDouble() - pidPositions.get(id.id)) < 5)) { // TODO configure deadzone
          usePID.put(id.id, false);
          velocityRequests.get(id.id).Velocity = 0;
          motors.get(id.id).setVoltage(0.0);
        }
      }
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