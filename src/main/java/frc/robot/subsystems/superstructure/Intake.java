// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicDutyCycle_Velocity;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;


public class Intake extends SubsystemBase {

  private final TalonFX leftAngle;
  private final TalonFX rightAngle;

  private final CANcoder angleEncoder;

  private final SparkMax intakeMotor;

  private final TrapezoidProfile.Constraints constraints =
  new TrapezoidProfile.Constraints(
    0.3,
    1
  );

  private final ProfiledPIDController pidController =
  new ProfiledPIDController(
    0.01,
    0.0,
    0.0,
    constraints
  );

  private final ArmFeedforward intakeFF =
  new ArmFeedforward(
    0.01,
    0.0,
    0.0
  );

  private final DynamicMotionMagicVoltage magic =
  new DynamicMotionMagicVoltage(
    0.0,
    0.0,
    0.0,
    0.0
  );


  // Add with other instance variables
  private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  
  // Control buttons
  private final GenericEntry intakeInButton, intakeOutButton, intakeStopButton;
  private final GenericEntry intakeSpeedSlider;
  
  // Status displays
  private final GenericEntry currentAngleDisplay;
  private final GenericEntry motorCurrentDisplay;
  
  // Angle control buttons
  private final GenericEntry angleUpButton, angleDownButton;
  private final GenericEntry angleSpeedSlider;
  private final GenericEntry anglePositionDisplay;
  
  /** Creates a new Intake. */
  public Intake() {
    leftAngle = new TalonFX(37);
    rightAngle = new TalonFX(38);
    angleEncoder = new CANcoder(39);
    intakeMotor = new SparkMax(35, MotorType.kBrushless);

    // Configure TalonFX motors
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    angleConfig.Voltage.PeakForwardVoltage = 12.0;
    angleConfig.Voltage.PeakReverseVoltage = -12.0;
    angleConfig.CurrentLimits.SupplyCurrentLimit = 40;
    angleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    var slot0config = angleConfig.Slot0;
    slot0config.kP = 0.1;
    slot0config.kI = 0.0;
    slot0config.kD = 0.0;
    slot0config.kG = 0.56;
    slot0config.kS = 0.0;
    slot0config.kV = 1.62;
    slot0config.kA = 0.03;

    leftAngle.getConfigurator().apply(angleConfig);
    rightAngle.getConfigurator().apply(angleConfig);

    rightAngle.setControl(new Follower(37, true));
    
    // Configure CANcoder
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    angleEncoder.getConfigurator().apply(encoderConfig);

    // Configure SparkMax
    SparkMaxConfig neo550Config = new SparkMaxConfig();


    neo550Config
        .smartCurrentLimit(20)  
        .idleMode(IdleMode.kCoast)  
        .voltageCompensation(12.0)  
        .openLoopRampRate(0.1);     
    
    // Apply our configuration with proper timeout
    intakeMotor.setCANTimeout(250);
    intakeMotor.configure(neo550Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize control buttons
    intakeInButton = intakeTab.add("Intake In", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
        
    intakeOutButton = intakeTab.add("Intake Out", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(1, 0)
        .withSize(1, 1)
        .getEntry();
        
    intakeStopButton = intakeTab.add("Stop Intake", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(2, 0)
        .withSize(1, 1)
        .getEntry();
        
    // Speed control slider
    intakeSpeedSlider = intakeTab.add("Intake Speed", 0.5)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 1.0))
        .withPosition(0, 1)
        .withSize(2, 1)
        .getEntry();
        
    // Status displays
    currentAngleDisplay = intakeTab.add("Current Angle", 0.0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 2)
        .withSize(1, 1)
        .getEntry();
        
    motorCurrentDisplay = intakeTab.add("Motor Current", 0.0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(1, 2)
        .withSize(1, 1)
        .getEntry();

    // Angle control buttons
    angleUpButton = intakeTab.add("Angle Up", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(0, 3)
        .withSize(1, 1)
        .getEntry();
        
    angleDownButton = intakeTab.add("Angle Down", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(1, 3)
        .withSize(1, 1)
        .getEntry();
        
    // Angle speed control
    angleSpeedSlider = intakeTab.add("Angle Speed", 0.3)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 1.0))
        .withPosition(0, 4)
        .withSize(2, 1)
        .getEntry();
        
    // Angle position display
    anglePositionDisplay = intakeTab.add("Angle Position", 0.0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 3)
        .withSize(1, 1)
        .getEntry();
  }

  @Override
  public void periodic() {
    // Get current speed setting
    double speed = intakeSpeedSlider.getDouble(0.5);
    
    // Handle button inputs
    if (intakeInButton.getBoolean(false)) {
        intakeMotor.set(speed);
    } else if (intakeOutButton.getBoolean(false)) {
        intakeMotor.set(-speed);
    } else if (intakeStopButton.getBoolean(false)) {
        intakeMotor.set(0);
    }
    
    // Update displays
    currentAngleDisplay.setDouble(angleEncoder.getAbsolutePosition().getValueAsDouble());
    motorCurrentDisplay.setDouble(intakeMotor.getOutputCurrent());

    
    if (angleUpButton.getBoolean(false)) {
        moveAngle(0.2);
    } else if (angleDownButton.getBoolean(false)) {
        moveAngle(-0.2);
    } else {
        moveAngle(0);  // Stop movement
    }
    
    // Update angle display
    anglePositionDisplay.setDouble(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public void moveAngle(double speed) {
    magic.Velocity = speed;
    leftAngle.setControl(magic);
  }
}
