// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PIDSS extends SubsystemBase {

  public final SparkMax Motor1 = new SparkMax(9, MotorType.kBrushless);
  public final SparkMax Motor2 = new SparkMax(10, MotorType.kBrushless);
  public final SparkMax Algae = new SparkMax(11, MotorType.kBrushless);

  private final PIDController Velo_PID = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  private final PIDController Pos_PID = new PIDController(Constants.kP, Constants.kI, Constants.kD);

  private boolean manual = true;

  private double desiredVelocity;
  private double desiredPosition;

  private double voltage;

  public PIDSS() {
    Motor2.setInverted(true);
    Motor2.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
