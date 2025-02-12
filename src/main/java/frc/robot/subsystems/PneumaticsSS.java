// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSS extends SubsystemBase {
  /** Creates a new PneumaticsSS. */
  public final DoubleSolenoid pneumaticTest;

  public PneumaticsSS() {
    pneumaticTest =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            Constants.PneumaticsConstants.forwardendSol,
            Constants.PneumaticsConstants.reverseendSol);
    pneumaticTest.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pneumaticTest();
    SmartDashboard.putData("Test Status", pneumaticTest);
  }

  public void Toggle() {
    pneumaticTest.toggle();
    ;
  }

  public void pneumaticTest() {
    pneumaticTest.get();
  }

  public void Reverse() {
    pneumaticTest.set(Value.kReverse);
  }
}
