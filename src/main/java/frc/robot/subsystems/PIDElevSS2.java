package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class PIDElevSS2 extends SubsystemBase {

  public final SparkMax Motor2 = new SparkMax(10, MotorType.kBrushless);


  private final PIDController Velo_PID = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  private final PIDController Pos_PID = new PIDController(Constants.kP, Constants.kI, Constants.kD);

  private boolean manual = true;

  private double desiredVelocity;
  private double desiredPosition;

  private double voltage;

  public void MotorStop() {
    Motor2.set(0);
  }

  public void Zero() {
    Motor2.set(-0.3);
  }

  // Setup for the PID SS
  public PIDElevSS2() {
    Motor2.setInverted(true);
    Motor2.getEncoder().setPosition(0);
  }

  public void AutoMotor(double speed) {
    Motor2.set(speed);
  }

  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Desired Elevator Velocity", desiredVelocity);
    SmartDashboard.putNumber("Elevator Voltage", getVoltage());

    setVoltage();
  }

  public void setVoltage() {
    double PIDVoltage;
    if (manual) { // Calculate position, if it is the position, stop the arm
      PIDVoltage = Velo_PID.calculate(getVelocity(), desiredVelocity);
    } else {
      // Otherwise try to go to the position
      double PIDVelocity = Pos_PID.calculate(getPosition(), desiredPosition);
      PIDVoltage = Velo_PID.calculate(getVelocity(), PIDVelocity);
      // double feedforwardVoltage = feedforward.calculate(getPosition(), 10);
      // PIDVoltage = PIDVoltage + feedforwardVoltage;
    }

    voltage = PIDVoltage;

    Motor2.setVoltage(PIDVoltage);
  }

  public double getVoltage() {
    return voltage;
  }

  public void setVelocity(double degreesPerSecond) {
    manual = true;

    desiredVelocity = degreesPerSecond;
  }

  public void setPosition(double degrees) {
    manual = false;

    desiredPosition = degrees;
  }

  public double getPosition() {
    return Motor2.getEncoder().getPosition() * 8;
  }

  public double getVelocity() {
    return Motor2.getEncoder().getVelocity() * (2 / 15);
  }
}
