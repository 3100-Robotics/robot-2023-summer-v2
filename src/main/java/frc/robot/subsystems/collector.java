package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision;
import frc.robot.Constants.collectorConstants;
import frc.robot.Constants.collectorStates;;

public class collector extends SubsystemBase{

    private final CANSparkMax angleMotor;
    private final AbsoluteEncoder angleEncoder;

    private final CANSparkMax shooter1;
    private final CANSparkMax shooter2;

    private final LinearFilter currentFilter = LinearFilter.movingAverage(20);

    private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);
    private PIDController PID = new PIDController(collectorConstants.angleP,
                                                    collectorConstants.angleI,
                                                    collectorConstants.angleD);

    private vision Vision;

    public double setpoint = 0;

    private collectorStates state = collectorStates.nothing;

    private double[] shooterData = new double[] {0, 0};

    public collector(vision Vision) {
        angleMotor = new CANSparkMax(collectorConstants.angleMotorPort, MotorType.kBrushless);

        shooter1 = new CANSparkMax(collectorConstants.leftShooterPort, MotorType.kBrushless);
        shooter2 = new CANSparkMax(collectorConstants.rightShooterPort, MotorType.kBrushless);

        // set the encoder to be the connected absolute encoder
        angleEncoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        PID.setSetpoint(collectorConstants.collectorHomeAngle);
        PID.setTolerance(1);

        // set up the pid controller

        // define the zero offset
        angleEncoder.setInverted(true);
        angleEncoder.setZeroOffset(collectorConstants.collectorOffset*360);
        angleEncoder.setPositionConversionFactor(360);

        this.Vision = Vision;

        configureMotors();
    }

    private void configureMotors() {
        // configure the motor
        angleMotor.setInverted(false);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setSmartCurrentLimit(25);
        // to enable when I have the correct number of rotations
        angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)0.35);
        angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        angleMotor.burnFlash();

        shooter2.follow(shooter1, true);
        shooter1.setIdleMode(IdleMode.kCoast);
        shooter2.setIdleMode(IdleMode.kCoast);
        shooter1.setSmartCurrentLimit(40);
        shooter2.setSmartCurrentLimit(40);
        shooter1.setInverted(true);
        shooter1.burnFlash();
        shooter2.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter anlge", angleEncoder.getPosition());
        // SmartDashboard.putNumber("max accel", angleController.getSmartMotionMaxAccel(0));
        // SmartDashboard.putNumber("max vel", angleController.getSmartMotionMaxVelocity(0));
        // SmartDashboard.putNumber("collector p", angleController.getP());
        // SmartDashboard.putNumber("collector I", angleController.getI());
        // SmartDashboard.putNumber("collector D", angleController.getD());
        // SmartDashboard.putNumber("collector FF", angleController.getFF());
        angleMotor.setVoltage(PID.calculate(angleEncoder.getPosition()) + feedforward.calculate(setpoint, 0));
    }

    public Command setAngle(double angle) {
        return this.runOnce(() -> { PID.setSetpoint(angle);
                                    this.setpoint=angle;
                                    this.state = collectorStates.moving;}).
                andThen(Commands.waitUntil(this::atAngleSetpoiont).
                andThen(this.runOnce(() -> state = collectorStates.nothing)));
    }

    public Boolean atAngleSetpoiont() {
        return PID.atSetpoint();
    }

    public Command setSpeed(DoubleSupplier speed) {
        return this.runOnce(() -> shooter1
        .set(speed.getAsDouble()));
    }

    public Command setSpeed(double speed) {
        return this.runOnce(() -> shooter1.set(speed));
    }

    public Command collect() {
        return Commands.sequence(
            setAngle(collectorConstants.collectorCollectAngle),
            Commands.runOnce(() -> state = collectorStates.collecting),
            setSpeed(-0.4).andThen(Commands.waitUntil(() -> currentFilter.calculate(shooter1.getOutputCurrent()) > 20)),
            stopShooterCommand(), 
            Commands.runOnce(currentFilter::reset), 
            setAngle(collectorConstants.collectorHomeAngle));
    }

    public Command collectInplace() {
        return Commands.sequence(
            Commands.runOnce(() -> state = collectorStates.collecting),
            setSpeed(-0.4).andThen(Commands.waitUntil(() -> currentFilter.calculate(shooter1.getOutputCurrent()) > 20)),
            stopShooterCommand(), 
            Commands.runOnce(currentFilter::reset));
    }

    private Command stopShooterCommand() {
        return this.runOnce(() -> {shooter1.stopMotor(); state = collectorStates.nothing;});
    }

    public Command shoot(Constants.shelves shelf) {
        return Commands.sequence(
            this.runOnce(() -> obtainShooterData(shelf)),
            setAngle(shooterData[0]),
            Commands.runOnce(() -> state = collectorStates.shooting),
            setSpeed(shooterData[1]),
            Commands.runOnce(() -> state = collectorStates.moving),
            setAngle(Constants.collectorConstants.collectorHomeAngle),
            Commands.runOnce(() -> state = collectorStates.nothing));
    }

    private void obtainShooterData(Constants.shelves shelf) {
        shooterData = Vision.getShooterData(shelf);
    }

    public Constants.collectorStates getState() {
        return state;
    }
}
