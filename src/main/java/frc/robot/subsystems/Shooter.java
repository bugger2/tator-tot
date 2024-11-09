package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private SparkMax motor;

    public Shooter() {
        motor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);

        SparkBaseConfig motorConfig = new SparkBaseConfig() {}
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(25);
    }

    public Command shoot() {
        return this.run(() -> motor.setVoltage(Volts.of(9)));
    }

    public Command stop() {
        return this.run(() -> motor.setVoltage(Volts.of(0)));
    }
}
