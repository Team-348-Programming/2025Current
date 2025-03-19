// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// Access the hidden files
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.equation.Variable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final PneumaticsSS rc_PneumaticsSS = new PneumaticsSS();
  public static final PIDSS rc_PIDSS = new PIDSS();
  public static final CoralRemoveSS rc_coralRemoveSS = new CoralRemoveSS();

  //NamedCommands.registerCommand("ElevUp", PneumaticsSS.ToggleCoral());

  // The robot's commands
  public static final PneumaticsC rc_PneumaticsC = new PneumaticsC(rc_PneumaticsSS);
  public static final ManualZeroC rc_manualZeroC = new ManualZeroC(rc_PIDSS);
  public static final ClimberC rc_climberC = new ClimberC(rc_PneumaticsSS);
  public static final TrapdoorC rc_trapdoorC = new TrapdoorC(rc_PneumaticsSS);

  // Other instantiations
  public static final PneumaticHub PH = new PneumaticHub(1);

  // The controllers
  public static final CommandXboxController m_driverController = new CommandXboxController(0);
  public static final CommandXboxController m_operatorController = new CommandXboxController(1);

  // Camera
  public static final UsbCamera CoralCamera = CameraServer.startAutomaticCapture("Coral", 1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("ElevUp", new Command() {
      @Override
      public void initialize() {
          rc_PneumaticsSS.ToggleCoral();
      }
    });

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // Driver Controller
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());
    m_driverController.leftTrigger().whileTrue(rc_trapdoorC);
    
    // Operator Controller
    // Pneumatics
    m_operatorController.a().whileTrue(rc_PneumaticsC); // Coral
    // Elevator PID
    m_operatorController.povUp().onTrue(new ElevPIDC(rc_PIDSS, () -> 31)); // 32 MAX HEIGHT
    m_operatorController.povRight().onTrue(new ElevPIDC(rc_PIDSS, () -> 13.5));
    m_operatorController.povDown().onTrue(new ElevPIDC(rc_PIDSS, () -> -0.5));
    m_operatorController.start().whileTrue(rc_manualZeroC);

    m_operatorController.y().whileTrue(new CoralRemoveC(rc_coralRemoveSS, -0.5));
    m_operatorController.x().whileTrue(new CoralRemoveC(rc_coralRemoveSS, 0.5));

    m_operatorController.b().onTrue(rc_climberC);

    m_operatorController.leftTrigger().whileTrue(rc_trapdoorC);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Run path following command, then stop at the end.
    //return new PathPlannerAuto("Auto Test");

    // Backup Auto
    return new Auto(m_robotDrive);
  }
}

