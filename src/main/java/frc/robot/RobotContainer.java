// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.autos.SimpleCoralAuto;
import frc.robot.commands.AlgieInCommand;
import frc.robot.commands.AlgieOutCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.CoralOutCommand;
import frc.robot.commands.CoralStackCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public final RollerSubsystem m_roller = new RollerSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final DriveSubsystem m_drive = new DriveSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();

  public final SimpleCoralAuto m_simpleCoralAuto = new SimpleCoralAuto(m_drive, m_roller, m_arm);
  public final DriveForwardAuto m_driveForwardAuto = new DriveForwardAuto(m_drive);

  private final DifferentialDrive myDrive;
  private final Joystick driveStick;
  private final Joystick exampleJoystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    myDrive = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    driveStick = new Joystick(0);
    Trigger driveTrigger = new Trigger(() -> driveStick.getRawButton(1));
    configureBindings();
    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    m_chooser.setDefaultOption("Coral Auto", m_simpleCoralAuto);
    m_chooser.addOption("Drive Forward Auto", m_driveForwardAuto);
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    myDrive.arcadeDrive(-driveStick.getY(), -driveStick.getX());

    driveTrigger.whileTrue(new DriveCommand(m_drive, 
    () -> -m_driverController.getLeftY() * DriveConstants.SLOW_MODE_MOVE,  
    () -> -m_driverController.getRightX() * DriveConstants.SLOW_MODE_TURN,
    () -> true));

    m_operatorController.rightBumper().whileTrue(new AlgieInCommand(m_roller));
    m_operatorController.rightTrigger(.2).whileTrue(new AlgieOutCommand(m_roller));
    m_operatorController.leftBumper().whileTrue(new ArmUpCommand(m_arm));
    m_operatorController.leftTrigger(.2).whileTrue(new ArmDownCommand(m_arm));
    m_operatorController.x().whileTrue(new CoralOutCommand(m_roller));
    m_operatorController.y().whileTrue(new CoralStackCommand(m_roller));
    m_operatorController.pov(0).whileTrue(new ClimberUpCommand(m_climber));
    m_operatorController.pov(180).whileTrue(new ClimberDownCommand(m_climber));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
}
}