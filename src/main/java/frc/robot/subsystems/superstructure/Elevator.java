// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;

public class Elevator extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  
  private static final double kG = 0.0;  // Voltage needed to fight gravity
  private static final double kDownSpeedMultiplier = 0.75; // Reduces down speed - adjust this!

  // Shuffleboard entries
  private final ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
  private final GenericEntry upButton = elevatorTab.add("Elevator Up", false)
      .withWidget("Toggle Button")
      .withPosition(0, 0)
      .withSize(1, 1)
      .getEntry();
      
  private final GenericEntry downButton = elevatorTab.add("Elevator Down", false)
      .withWidget("Toggle Button")
      .withPosition(1, 0)
      .withSize(1, 1)
      .getEntry();

  // Add these with other instance variables at the top
  private final GenericEntry speedEntry;
  private final GenericEntry positionEntry;
  private final GenericEntry leftRotationsEntry;
  private final GenericEntry rightRotationsEntry;

  // Add these variables at the top
  private double maxLeftRotations = 0.0;
  private double maxRightRotations = 0.0;
  private final GenericEntry maxLeftRotationsEntry;
  private final GenericEntry maxRightRotationsEntry;

  // Profiled PID Controller for smooth motionS
  private final TrapezoidProfile.Constraints constraints = 
      new TrapezoidProfile.Constraints(
          3.0,   // Max velocity in rotations per second
          2.0   // Max acceleration in rotations per second squared
      );
  
  private final ProfiledPIDController pidController = 
      new ProfiledPIDController(
          0.5,   // P gain
          0.00,   // I gain
          0.0,   // D gain
          constraints
      );

  private double targetPosition = 0.0;
  private final GenericEntry setPositionEntry;
  private final GenericEntry goToPositionButton;
  private boolean positionControl = false;

  // Add these with other instance variables
  private final GenericEntry voltageEntry;
  private final GenericEntry appliedOutputEntry;
  private final GenericEntry currentEntry;

  /** Creates a new ElevatorSubsystem. */
  public Elevator() {
    leftMotor = new SparkMax(25, MotorType.kBrushless);  // Update ID as needed
    rightMotor = new SparkMax(26, MotorType.kBrushless); // Update ID as needed
    
    
    configureNEO(leftMotor, false,true);  //master ccw positive
    configureNEO(rightMotor, true,true);  //slave cw positive
  

    //widgets
    speedEntry = elevatorTab.add("Elevator Speed", 0.0)
        .withPosition(0, 1)
        .withSize(2, 1)
        .getEntry();
    positionEntry = elevatorTab.add("Elevator Position (CANcoder)", 0.0)
        .withPosition(0, 2)
        .withSize(2, 1)
        .getEntry();
    leftRotationsEntry = elevatorTab.add("Left Motor Rotations", 0.0)
        .withPosition(0, 3)
        .withSize(2, 1)
        .getEntry();
    rightRotationsEntry = elevatorTab.add("Right Motor Rotations", 0.0)
        .withPosition(0, 4)
        .withSize(2, 1)
        .getEntry();

    maxLeftRotationsEntry = elevatorTab.add("Max Left Motor Rotations", 0.0)
        .withPosition(0, 5)
        .withSize(2, 1)
        .getEntry();
    maxRightRotationsEntry = elevatorTab.add("Max Right Motor Rotations", 0.0)
        .withPosition(0, 6)
        .withSize(2, 1)
        .getEntry();

    setPositionEntry = elevatorTab.add("Set Position", 0.0)
        .withPosition(2, 0)
        .withSize(1, 1)
        .getEntry();
        
    goToPositionButton = elevatorTab.add("Go To Position", false)
        .withWidget("Toggle Button")
        .withPosition(3, 0)
        .withSize(1, 1)
        .getEntry();

    // Configure PID Controller
    pidController.setTolerance(0.05); 

    // Add voltage monitoring widgets
    voltageEntry = elevatorTab.add("Bus Voltage", 0.0)
        .withPosition(0, 7)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kVoltageView)  // Nice voltage display!
        .getEntry();
        
    appliedOutputEntry = elevatorTab.add("Applied Output (Volts)", 0.0)
        .withPosition(2, 7)
        .withSize(2, 1)
        .getEntry();
        
    currentEntry = elevatorTab.add("Current Draw (Amps)", 0.0)
        .withPosition(4, 7)
        .withSize(2, 1)
        .getEntry();
  }

  private void configureNEO(SparkMax motor, boolean inverted, boolean softLimit) {
    SparkMaxConfig neoConfig = new SparkMaxConfig();
    
    // Create soft limit config for elevator
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig
        .forwardSoftLimit(165)     // Adjust these limits for your elevator!
        .forwardSoftLimitEnabled(softLimit)
        .reverseSoftLimit(0.0)     // Bottom position
        .reverseSoftLimitEnabled(softLimit);
    
    neoConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)  // Use brake mode for elevator
        .voltageCompensation(12.0)
        .openLoopRampRate(0.1)
        .apply(softLimitConfig)
        .inverted(inverted)
        .disableFollowerMode();

    // Add follow configuration for right motor
    // if (motor == rightMotor) {
    //     neoConfig.follow(leftMotor);
    // }
    
    motor.setCANTimeout(250);
    motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0.0);  // Reset encoder to zero
  }

  private void configureCANcoder(CANcoder canCoder) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    
    // Configure the sensor
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 
    
    // Apply configuration
    canCoder.getConfigurator().apply(config);
    canCoder.setPosition(0.0);
  }

  /** 
   * Sets the elevator speed. Positive values move up, negative values move down.
   * Includes gravity compensation when moving up and speed reduction when moving down! ^w^
   * @param speed Speed from -1.0 to 1.0
   */
  public void setElevatorSpeed(double speed) {
    // Add gravity feedforward when moving up
    double gravityCompensation = kG;
    
    // Reduce speed when moving down
    if (speed < 0) {
      speed *= kDownSpeedMultiplier;
    }
    
    leftMotor.set(speed + gravityCompensation);
    rightMotor.set(speed + gravityCompensation);
  }

  /**
   * Move the elevator up at a fixed speed
   */
  public void up() {
    setElevatorSpeed(0.2);  // Adjust this value based on your needs!
  }

  /**
   * Move the elevator down at a fixed speed
   */
  public void down() {
    setElevatorSpeed(-0.2);  // Adjust this value based on your needs!
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    setElevatorSpeed(0.0);
  }

    public void setPosition(double position) {
    targetPosition = position;
    pidController.setGoal(position);
  }

  public void disablePositionControl() {
    positionControl = false;
  }

  public boolean atTargetPosition() {
    return pidController.atGoal();
  }

  @Override
  public void periodic() {
    // Check for negative position and reset if needed
    // if (leftMotor.getEncoder().getPosition() < 0) {
    //   leftMotor.getEncoder().setPosition(0.0);
    // }
    // if (rightMotor.getEncoder().getPosition() < 0) {
    //   rightMotor.getEncoder().setPosition(0.0);
    // }

    // Handle position control
    // if (goToPositionButton.getBoolean(false)) {
    //   setPosition(setPositionEntry.getDouble(0.0));
    // }
    
    // if (positionControl) {
    //   setPosition(setPositionEntry.getDouble(0.0));
    // } else {
      // Normal button control
      if (upButton.getBoolean(false)) {
        up();
      } else if (downButton.getBoolean(false)) {
        down();
      } else {
        stop();
      }
    // }

    // Update values instead of creating new widgets
    speedEntry.setDouble(leftMotor.get());
    positionEntry.setDouble(leftMotor.getEncoder().getPosition());
    leftRotationsEntry.setDouble(leftMotor.getEncoder().getPosition());
    rightRotationsEntry.setDouble(rightMotor.getEncoder().getPosition());

    // Track maximum rotations
    double leftRotations = Math.abs(leftMotor.getEncoder().getPosition());
    double rightRotations = Math.abs(rightMotor.getEncoder().getPosition());
    
    maxLeftRotations = Math.max(maxLeftRotations, leftRotations);
    maxRightRotations = Math.max(maxRightRotations, rightRotations);

    // Update max rotation displays
    maxLeftRotationsEntry.setDouble(maxLeftRotations);
    maxRightRotationsEntry.setDouble(maxRightRotations);

    // Update voltage telemetry
    voltageEntry.setDouble(leftMotor.getBusVoltage());
    appliedOutputEntry.setDouble(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    currentEntry.setDouble(leftMotor.getOutputCurrent());
  }
}


    
    

