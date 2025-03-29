// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class PneumaticsSS extends SubsystemBase {
  /** Creates a new PneumaticsSS. */
  public final DoubleSolenoid Coral;
  public final DoubleSolenoid Climb;
  public final DoubleSolenoid Door;

  public static String DoorStatus = "null";

  public PneumaticsSS() {
    Coral =
            new DoubleSolenoid(
                PneumaticsModuleType.REVPH,
                Constants.PneumaticsConstants.forwardCoralSol,
                Constants.PneumaticsConstants.reverseCoralSol);
    Climb =
            new DoubleSolenoid(
                PneumaticsModuleType.REVPH,
                Constants.PneumaticsConstants.forwardClimbSol,
                Constants.PneumaticsConstants.reverseClimbSol);
    Door = 
            new DoubleSolenoid(
                PneumaticsModuleType.REVPH,
                Constants.PneumaticsConstants.forwardDoorSol,
                Constants.PneumaticsConstants.reverseDoorSol);
    Coral.set(Value.kReverse);
    Climb.set(Value.kForward);
    Door.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ToggleCoral() {
    Coral.toggle();
  }

  public void ToggleClimber() {
    Climb.toggle();
  }

  public void ToggleDoor() {
    Door.set(Value.kForward);
  }
}
