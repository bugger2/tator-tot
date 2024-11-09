package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
    // variable declarations these are values that you can reference throughout
    // this file, of specific types that allows you to do different things with
    // them. For example, motor is of type SparkMax, so you can tell it to do
    // motor things.  `private` says that only this file can reference these
    // variables, and it is generally good practive to make all variables at the
    // top level of a file private
    private SparkMax motor;
    private DigitalInput sensor;

    public final Trigger gamePieceDetected;

    public Intake() {
        // creates a new SparkMax object and assigns it to the `motor`
        // variable. Motor has an ID of 1, and is specified as brushless
        motor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);

        // creates a default configuration for the motor with two notable
        // exceptions:
        //
        // brake mode is enabled, so the motor will have an active braking force
        // applied when not in use
        // 
        // A current limit of 25 amps is applied, so the motor will not exceed
        // this limit and burn out
        SparkBaseConfig motorConfig = new SparkBaseConfig() {}
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(25);

        // flashes the configuration to the motor controller
        // Also tells the motor controller to completely reset, and for the
        // settings to persist after reboot
        motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // constructs the sensor object
        sensor = new DigitalInput(0);

        gamePieceDetected = new Trigger(sensor::get);
    }

    /**
     * @return A {@link Command} that will run the motor forward until the sensor is hit
     */
    public Command intake() {
        return this.run(() -> motor.setVoltage(Volts.of(6)))
            .until(sensor::get);
    }

    /**
     * @return A {@link Command} that will run the motor forward until the sensor is dropped
     */
    public Command feed() {
        return this.run(() -> motor.setVoltage(Volts.of(6)))
            .until(() -> !sensor.get())
            .finallyDo(() -> motor.setVoltage(Volts.of(0)));
    }

    /**
     * @return A {@link Command} that will stop the motor
     */
    public Command stop() {
        return this.run(() -> motor.setVoltage(Volts.of(0)));
    }
}
