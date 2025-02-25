package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.CANcoder;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

public class Grabber extends SubsystemBase {
    private final SparkMax rightIntake;
    private final SparkMax leftIntake;
    private final SparkMax rightangle;
    private final SparkMax leftangle;
    private final CANcoder grabberEncoder;
    private static final double DEFAULT_KG = 0.00;
    private static final double cancderoffset = 0.2;
    private int startupCounter = 0;
    private int stallCounter = 0;

    // Shuffleboard entries
    private final ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");
    private final GenericEntry forwardButton, backwardButton;
    private final GenericEntry angleDisplay;
    private final GenericEntry bothInButton, bothOutButton;
    private final GenericEntry upRPMStatus, downRPMStatus;
    private final GenericEntry defaultButton, coralStationButton, l4Button, l23Button;

    // private final ShuffleboardTab motorTab = Shuffleboard.getTab("Motor Controls");

    private final ProfiledPIDController pidController;
    private boolean positionControl = false;
    private double targetPosition = 0.0;

    // Add with other instance variables
    private final TrapezoidProfile.Constraints constraints = 
        new TrapezoidProfile.Constraints(
            0.1,   // Max velocity in rotations per second
            0.15    // Max acceleration in rotations per second squared
        );
    
    private final ArmFeedforward grabberFF = 
    new ArmFeedforward(
        0.01, 
        0.01, 
        0
    );
    
    private final DigitalInput intakeLimitSwitch;

    public Grabber() {
        rightIntake = new SparkMax(28, MotorType.kBrushless);
        leftIntake = new SparkMax(29, MotorType.kBrushless);
        leftangle = new SparkMax(35, MotorType.kBrushless);
        rightangle = new SparkMax(36, MotorType.kBrushless);
        grabberEncoder = new CANcoder(27, "rio");

        intakeLimitSwitch = new DigitalInput(9);

        // Initialize all Shuffleboard widgets
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

        angleDisplay = grabberTab.add("Current Angle", 0.0)
            .withWidget("Text View")
            .withPosition(2, 4)
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

        // Initialize PID controller
        pidController = new ProfiledPIDController(
            8.7,    // kP
            0.003,    // kI 
            0.005,    // kD
            constraints  // Motion constraints
        );



        pidController.reset(grabberEncoder.getAbsolutePosition().getValueAsDouble());
        pidController.setTolerance(0.0004);  // Degrees of acceptable error
        pidController.setIZone(0.05);  
        pidController.disableContinuousInput();
        pidController.setIntegratorRange(0,0);
        pidController.setGoal(grabberEncoder.getAbsolutePosition().getValueAsDouble());
        pidController.calculate(grabberEncoder.getAbsolutePosition().getValueAsDouble());
        // Add position preset buttons
        defaultButton = grabberTab.add("default", false)
            .withWidget("Toggle Button")
            .withPosition(0, 7)
            .withSize(1, 1)
            .getEntry();
            
        l4Button = grabberTab.add("l4", false)
            .withWidget("Toggle Button")
            .withPosition(1, 7)
            .withSize(1, 1)
            .getEntry();
            
        coralStationButton = grabberTab.add("coral station", false)
            .withWidget("Toggle Button")
            .withPosition(2, 7)
            .withSize(1, 1)
            .getEntry();
        
        l23Button = grabberTab.add("l23", false)
            .withWidget("Toggle Button")
            .withPosition(3, 3)
            .withSize(1, 1)
            .getEntry();

        configureNEO550(rightIntake);
        configureNEO550(leftIntake);
        configureNEO(leftangle);    // Configure leader first
        
        // Configure follower
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .follow(leftangle, true);  // Set to follow leftangle
            
        rightangle.setCANTimeout(250);
        rightangle.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        leftangle.getEncoder().setPosition(0.0);

    }

    @Override
    public void periodic() {
        double baseKG = DEFAULT_KG;
        double currentAngle = grabberEncoder.getAbsolutePosition().getValueAsDouble();
        
        // Calculate kG based on angle (now in volts)
        double kG = (currentAngle <= 0) ? -baseKG * 12.0 : baseKG * 12.0;
    
        // Check position buttons
        if (defaultButton.getBoolean(false)) {
            setPosition(-0.0432);  // Default position
        } else if (l4Button.getBoolean(false)) {
            setPosition(-0.1284);  // l4 pos  l2 l3 is -0.1884
        } else if (coralStationButton.getBoolean(false)) {
            setPosition(0.21630859375);  // Coral station position
        }else if (l23Button.getBoolean(false)) {
            setPosition(-0.1884);
        }else if (forwardButton.getBoolean(false)) {
            set(0.069);
        } else if (backwardButton.getBoolean(false)) {
            set(-0.069);
        }else {
                leftangle.setVoltage(kG); 
        }
        


        
           

        // Update angle display
        angleDisplay.setDouble(currentAngle);


        // Handle synchronized motor control
            if (bothInButton.getBoolean(false)) {
                intake();
        } else if (bothOutButton.getBoolean(false)) {
            placeCoral();
        } else {
            leftIntake.set(0);
            rightIntake.set(0);
        }

        // Check RPM and update status
        double rightIntakeRPM = Math.abs(rightIntake.getEncoder().getVelocity());
        double leftIntakeRPM = Math.abs(leftIntake.getEncoder().getVelocity());
        
        SmartDashboard.putNumber("pid", pidController.calculate(grabberEncoder.getAbsolutePosition().getValueAsDouble()));
        SmartDashboard.putBoolean("limit switch", intakeLimitSwitch.get());
    }

    // Copy your configuration methods
    private void configureNEO550(SparkMax motor) {
        SparkMaxConfig neo550Config = new SparkMaxConfig();
    
        neo550Config
            .smartCurrentLimit(30)  
            .idleMode(IdleMode.kCoast)  
            .voltageCompensation(12.0)  
            .openLoopRampRate(0.1);     
    
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
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            // .openLoopRampRate(0.1)
            .apply(softLimitConfig)
            .inverted(false);   
    
        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the target position for the grabber
     * @param position Target angle in degrees
     * @return true if position is within valid range
     */
    public void setPosition(double position) {
        // pidController.reset(grabberEncoder.getAbsolutePosition().getValueAsDouble());
        pidController.setGoal(position);
        set(MathUtil.clamp(pidController.calculate(grabberEncoder.getAbsolutePosition().getValueAsDouble()), -0.08, 0.08));
    }

    /**
     * Sets the angle motor output based on voltage
     * clamped between -1 ~ 1
     * @param output Target voltage percentage 
     */
    public void set(double output) {
        double currentAngle = grabberEncoder.getAbsolutePosition().getValueAsDouble();
        
        // Convert position to radians for ArmFeedforward
        double positionRadians = (currentAngle - cancderoffset) * Math.PI * 2;  // Adjust scaling as needed
        
        // Calculate feedforward voltage
        double ffVolts = grabberFF.calculate(positionRadians, output);
        
        // Combine feedforward with commanded output
        double totalVoltage = (MathUtil.clamp(output + ffVolts, -1, 1) * 12.0);
        
        leftangle.setVoltage(totalVoltage);
    }

    public double calculateKG(double position) {
        double kG = Math.cos((position - 0.2) * 15.5) * 0.01 + 0.01;
        return kG;
    }
    
    /**
     * @return true if grabber is at the target position
     */
    public boolean atPosition() {
        return pidController.atSetpoint();
    }
    
    /**
     * @return current angle of the grabber in degrees
     */
    public double getCurrentAngle() {
        return grabberEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void intake() {
        if (!intakeLimitSwitch.get()) {
            rightIntake.set(0);
            leftIntake.set(0);
            
        }else{
        rightIntake.set(-0.4);
        leftIntake.set(0.4);
        }
    }

    public boolean isIntakeStopped() {
        return intakeLimitSwitch.get();
    }

    public void placeCoral() {
        rightIntake.set(0.2);
        leftIntake.set(-0.2);
    }
} 