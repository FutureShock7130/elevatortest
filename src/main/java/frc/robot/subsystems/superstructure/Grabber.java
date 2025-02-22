package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import java.util.Map;
import edu.wpi.first.math.controller.PIDController;

public class Grabber extends SubsystemBase {
    private final SparkMax rightIntake;
    private final SparkMax leftIntake;
    private final SparkMax rightangle;
    private final SparkMax leftangle;
    private final CANcoder grabberEncoder;
    private static final double DEFAULT_KG = 0.0;
    private int startupCounter = 0;
    private int stallCounter = 0;

    // Shuffleboard entries
    private final ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");
    private final GenericEntry upButton, downButton, upSpeed, downSpeed;
    private final GenericEntry forwardButton, backwardButton, flatButton;
    private final GenericEntry angleDisplay, kGTuner;
    private final GenericEntry bothInButton, bothOutButton;
    private final GenericEntry upRPMStatus, downRPMStatus;

    // private final ShuffleboardTab motorTab = Shuffleboard.getTab("Motor Controls");

    private final PIDController pidController = new PIDController(
        0.07,   // kP
        0.035,  // kI - helps eliminate steady-state error UwU
        0.007   // kD - reduces overshoot and oscillation ✨
    );

    public Grabber() {
        rightIntake = new SparkMax(28, MotorType.kBrushless);
        leftIntake = new SparkMax(29, MotorType.kBrushless);
        leftangle = new SparkMax(30, MotorType.kBrushless);
        rightangle = new SparkMax(31, MotorType.kBrushless);
        grabberEncoder = new CANcoder(27, "rio");

        // Initialize all Shuffleboard widgets
        upButton = grabberTab.add("Up Motor", false)
            .withWidget("Toggle Button")
            .withPosition(0, 0)
            .getEntry();

        downButton = grabberTab.add("Down Motor", false)
            .withWidget("Toggle Button")
            .withPosition(0, 1)
            .getEntry();

        upSpeed = grabberTab.add("Up Motor Speed", 0.5)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", -1.0, "max", 1.0))
            .withPosition(1, 0)
            .getEntry();

        downSpeed = grabberTab.add("Down Motor Speed", 0.5)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", -1.0, "max", 1.0))
            .withPosition(1, 1)
            .getEntry();

        forwardButton = grabberTab.add("Turn Forward", false)
            .withWidget("Toggle Button")
            .withPosition(0, 4)
            .withSize(1, 1)
            .getEntry();

        backwardButton = grabberTab.add("Turn Back", false)
            .withWidget("Toggle Button")
            .withPosition(1, 4)
            .withSize(1, 1)
            .getEntry();

        flatButton = grabberTab.add("Flat", false)
            .withWidget("Toggle Button")
            .withPosition(2, 4)
            .withSize(1, 1)
            .getEntry();

        angleDisplay = grabberTab.add("Current Angle", 0.0)
            .withWidget("Text View")
            .withPosition(2, 4)
            .withSize(1, 1)
            .getEntry();

        kGTuner = grabberTab.add("Gravity Compensation", DEFAULT_KG)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", 0.0, "max", 0.2))
            .withPosition(3, 4)
            .withSize(1, 1)
            .getEntry();

        bothInButton = grabberTab.add("Both Motors In", false)
            .withWidget("Toggle Button")
            .withPosition(0, 5)
            .withSize(1, 1)
            .getEntry();

        bothOutButton = grabberTab.add("Both Motors Out", false)
            .withWidget("Toggle Button")
            .withPosition(1, 5)
            .withSize(1, 1)
            .getEntry();

        upRPMStatus = grabberTab.add("Up Motor RPM OK", true)
            .withWidget("Boolean Box")
            .withProperties(Map.of("colorWhenTrue", "Lime", "colorWhenFalse", "Red"))
            .withPosition(0, 6)
            .withSize(1, 1)
            .getEntry();

        downRPMStatus = grabberTab.add("Down Motor RPM OK", true)
            .withWidget("Boolean Box")
            .withProperties(Map.of("colorWhenTrue", "Lime", "colorWhenFalse", "Red"))
            .withPosition(1, 6)
            .withSize(1, 1)
            .getEntry();

        configureNEO550(rightIntake);
        configureNEO550(leftIntake);
        configureNEO(leftangle);    // Configure leader first
        
        // Configure follower
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .follow(leftangle, true);  // Set to follow leftangle
            
        rightangle.setCANTimeout(250);
        rightangle.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        leftangle.getEncoder().setPosition(0.0);
    }

    @Override
    public void periodic() {
        double kG = kGTuner.getDouble(DEFAULT_KG);
    
        if (forwardButton.getBoolean(false)) {
            leftangle.set(0.01 );  // Only need to control leader
            
          
            
        } else if (backwardButton.getBoolean(false)) {
            leftangle.set(-0.01);
            
           
            
        } else if (flatButton.getBoolean(false)) {
            leftangle.set(0.01 + kG);
            
            if (leftangle.getEncoder().getPosition() >= 1.71875) {
                leftangle.set(kG);
            }
            
        } else {
            leftangle.set(kG * 0.5);
        }

        // Get button states and speeds fow motow contwol OwO
        boolean upButtonState = upButton.getBoolean(false);
        boolean downButtonState = downButton.getBoolean(false);
        
        double upSpeedValue = upSpeed.getDouble(0.5);
        double downSpeedValue = downSpeed.getDouble(0.5);
        
        rightIntake.set(upButtonState ? upSpeedValue : 0);
        leftIntake.set(downButtonState ? downSpeedValue : 0);

        // Update angle display with new ratio
        double currentAngleDegrees = grabberEncoder.getAbsolutePosition().getValueAsDouble();  // Convert from rotations
        angleDisplay.setDouble(currentAngleDegrees);

        // Handle synchronized motor control
        if (bothInButton.getBoolean(false)) {
            rightIntake.set(-0.3);    // Up motor forward
            leftIntake.set(0.3); // Down motor reverse
        } else if (bothOutButton.getBoolean(false)) {
            double rightIntakeRPM = Math.abs(rightIntake.getEncoder().getVelocity());
            double leftIntakeRPM = Math.abs(leftIntake.getEncoder().getVelocity());
            
            if (startupCounter < 10) {  // Startup delay
                rightIntake.set(0.4);
                leftIntake.set(-0.4);
                startupCounter++;
            } else if (rightIntakeRPM < 50 || leftIntakeRPM < 50) {
                if (stallCounter < 50) {  // Wait ~0.5 seconds (25 * 20ms) before stopping
                    stallCounter++;
                    rightIntake.set(0.4);
                    leftIntake.set(-0.4);
                } else {
                    rightIntake.set(0);
                    leftIntake.set(0);
                    bothOutButton.setBoolean(false);
                    startupCounter = 0;
                    stallCounter = 0;
                }
            } else {
                stallCounter = 0;  // Reset stall counter if RPM is good
                rightIntake.set(0.4);
                leftIntake.set(-0.4);
            }
        } else {
            startupCounter = 0;  // Reset both counters when button released
            stallCounter = 0;
        }

        // Check RPM and update status
        double rightIntakeRPM = Math.abs(rightIntake.getEncoder().getVelocity());
        double leftIntakeRPM = Math.abs(leftIntake.getEncoder().getVelocity());
        
        SmartDashboard.putNumber("rpm", rightIntake.getEncoder().getVelocity());
    }

    // Copy your configuration methods
    private void configureNEO550(SparkMax motor) {
        SparkMaxConfig neo550Config = new SparkMaxConfig();
    
        neo550Config
            .smartCurrentLimit(40)  // Protect our smol motors! >w<
            .idleMode(IdleMode.kCoast)  // Better control!
            .voltageCompensation(12.0)  // Stable performance! UwU
            .openLoopRampRate(0.1);     // Smooth acceleration! 
    
        // Apply our configuration with proper timeout
        motor.setCANTimeout(250);
        motor.configure(neo550Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureNEO(SparkMax motor) {
        SparkMaxConfig neoConfig = new SparkMaxConfig();
    
    // Create soft limit config 
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
            .forwardSoftLimit(0.0)    
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit(0)       
            .reverseSoftLimitEnabled(false);
    
        neoConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .openLoopRampRate(0.1)
            .apply(softLimitConfig)
            .inverted(false);   // Flips motor direction
    
        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
} 