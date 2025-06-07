// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Motors.MotorType;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ElevatorConstants {
    public static final int PRIMARY_ELEVATOR_MOTOR_ID = 1;
    public static final int SECONDARY_ELEVATOR_MOTOR_ID = 2;
    public static final double HEIGHT_LEVEL_1 = 0.00001;
    public static final double HEIGHT_LEVEL_2 = 0.18;
    public static final double HEIGHT_LEVEL_3 = 0.38;
    public static final double HEIGHT_LEVEL_4 = 0.7;

    public static final ServoSubsystemConfiguration ElevatorConfiguration = new ServoSubsystemConfiguration(){{
     
      motorType = MotorType.talonFX;

      subsystemName = "Elevator";

      slaves = new MotorConstants[]{new MotorConstants(50,"rio",true,true)};

      master = new MotorConstants(51,"rio",true ,true);

      rotationsPerPositionUnit = 1.0 / (0.0363728 * Math.PI);

      sensorToMechanismRatio = 10.2857142857;
      
      pidContainerSlot0 = new PIDContainer(0, 1.2733, 0.060729, 0.334, 0.15, 0.0, 0.0);

      // pidContainerSlot1 = new PIDContainer(0.0, 0.0, 1.0, 0.15, 1000.0, 0.0, 0.0);
      //#region motion magic values
      motionMagicCruiseVelocity = 3;
      
      motionMagicAcceleration = 6;

      motionMagicJerk = 9;
      //#endregion motion magic values

      //#region cuurent limit
      supplyCurrentLimit = 40;

      enableSupplyCurrentLimit = true;

      statorCurrentLimit = 40;

      enableStatorCurrentLimit = true;
      //#endregion current limit

      //#region soft limits 
      forwardSoftLimit = 0.70;

      reverseSoftLimit = 0.0;
      //#endregion soft limits

      allowableError = 0.8;

      homePosition = 0.0;
    }};
  }
}
