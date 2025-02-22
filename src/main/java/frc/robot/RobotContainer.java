// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.superstructure.Grabber;
import frc.robot.subsystems.superstructure.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Declare subsystem
  private final Elevator m_elevatorSubsystem = new Elevator();
  private final Grabber m_Coral= new Grabber();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Joystick buttonbox1 = new Joystick(0);  // Update port number as needed
  private final Joystick buttonbox2 = new Joystick(1);  // Update port number as needed

  // Preset positions for elevator
  private static final double ELEVATOR_POSITION_GROUND = 0.0;
  private static final double ELEVATOR_POSITION_LOW = 2.0;    // Adjust these values!
  private static final double ELEVATOR_POSITION_MID = 4.0;    // Adjust these values!
  private static final double ELEVATOR_POSITION_HIGH = 6.0;   // Adjust these values!

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    // Register subsystem with Command Scheduler
    CommandScheduler.getInstance().registerSubsystem(m_elevatorSubsystem);
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
  private void configureButtonBindings() {
    // Elevator position presets on buttonbox1
    new JoystickButton(buttonbox1, 1)
        .onTrue(new InstantCommand(() -> 
            m_elevatorSubsystem.setPosition(ELEVATOR_POSITION_GROUND)));
            
    new JoystickButton(buttonbox1, 2)
        .onTrue(new InstantCommand(() -> 
            m_elevatorSubsystem.setPosition(ELEVATOR_POSITION_LOW)));
            
    new JoystickButton(buttonbox1, 3)
        .onTrue(new InstantCommand(() -> 
            m_elevatorSubsystem.setPosition(ELEVATOR_POSITION_MID)));
            
    new JoystickButton(buttonbox1, 4)
        .onTrue(new InstantCommand(() -> 
            m_elevatorSubsystem.setPosition(ELEVATOR_POSITION_HIGH)));

    // You can add more button bindings here for buttonbox1 (buttons 5-8)
    // and buttonbox2 (buttons 1-8)
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
