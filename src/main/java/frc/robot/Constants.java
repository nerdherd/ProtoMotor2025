// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.preferences.PrefDouble;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static PrefDouble kPosition = new PrefDouble("Position", 0.0);
  public static PrefDouble kRPM = new PrefDouble("RPM", 2.0);
  public static PrefDouble kP = new PrefDouble("P", 0);
  public static PrefDouble kI = new PrefDouble("I", 0);
  public static PrefDouble kD = new PrefDouble("D", 0);
  public static PrefDouble kV = new PrefDouble("V", 0);
  public static PrefDouble kVoltageLow = new PrefDouble("Voltage Low", 2.0);
  public static PrefDouble kVoltageHigh = new PrefDouble("Voltage High", 2.5); //TODO Voltage
  public static PrefDouble kVoltageIntervalHigh = new PrefDouble("Voltage Interval High", .1);
  public static PrefDouble kVoltageIntervalLow = new PrefDouble("Voltage Interval Low", 5);


  public enum MotorIDs {
    Motor1(0,1.0),
    Motor2(0,1.0),
    Motor3(0,1.0),
    Motor4(0,1.0),
    Motor5(0,1.0),
    Motor6(0,1.0),
    Motor7(0,1.0),
    Motor8(0,1.0),
    ;

    int id;
    double relspeed; // Relative speed to run at (1 is normal speed)
    MotorIDs(int _id,double _relspeed) {
      id = _id;
      relspeed = _relspeed;
    }
  }
}