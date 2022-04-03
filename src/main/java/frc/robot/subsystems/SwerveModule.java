package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final DutyCycleEncoder absoluteEncoder;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncolderOffsetRad;

    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, DutyCycleEncoder absoluteEncoder, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncolderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoder = absoluteEncoder;
        this.absoluteEncoder.reset();

        driveMotor = new TalonFX(driveMotorID);
        turningMotor = new TalonFX(turningMotorID);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); // Dont mess with I and D because the proportional term already does a good job
        turningPidController.enableContinuousInput(-Math.PI, Math.PI); //Tells that it is circular.

    }

    // Unit conversion methods
    public double driveEncoderPulseToDistance(double encoderPulse) {
        return encoderPulse * (2 * ModuleConstants.kWheelRadiusMeters * Math.PI) / ModuleConstants.kEncoderResolution / ModuleConstants.kDriveMotorGearRatio;
    }

    public double distanceToDriveEncoderPulse(double distance) {
        return distance * ModuleConstants.kEncoderResolution / (2 * ModuleConstants.kWheelRadiusMeters * Math.PI) * ModuleConstants.kDriveMotorGearRatio / 10;
    }

    public double turningEncoderPulseToRadians(double encoderPulse) {
        return encoderPulse / ModuleConstants.kEncoderResolution / ModuleConstants.kTurningMotorGearRatio * (2 * Math.PI);
    }

    public double radiansToTurningEncoderPulse(double angle) {
        return angle * ModuleConstants.kEncoderResolution * ModuleConstants.kTurningMotorGearRatio / (2 * Math.PI);
    }

    public double absoluteEncoderToRadians(double rawEncoder) {
        return (rawEncoder - absoluteEncolderOffsetRad) % 1 * (2 * Math.PI);
      }

    public double radiansToDegree(double radians){
        return radians *360 / (2 * Math.PI);
    }

    // Get Encoder Values
    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoderToRadians(absoluteEncoder.get());
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {

        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.Velocity, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getSourceChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(ControlMode.Velocity, 0);
        turningMotor.set(ControlMode.Velocity, 0);
    }

}
