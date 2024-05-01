package com.simulation;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class REVSimpleSimulation {

    private final NewDCMotorSim dcMotorSim;

    private final SimDouble simAppliedOutput;
    private final SimDouble simVelocity;
    private final SimDouble simPosition;
    private final SimDouble simCurrent;
    private final double dtSeconds;

    public REVSimpleSimulation(
            int deviceId,
            Measure<Voltage> kS,
            Measure<Per<Voltage, Velocity<Angle>>> kV,
            Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA,
            DCMotor gearbox,
            double gearing,
            double dtSeconds) {
        LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(
                kV.in(VoltsPerRadianPerSecond),
                kA.in(VoltsPerRadianPerSecondSquared));

        dcMotorSim = new NewDCMotorSim(plant, gearbox, gearing, kS.in(Volts));
        SimDeviceSim simDeviceSim = new SimDeviceSim(String.format("SPARK MAX [%s]", deviceId));
        simAppliedOutput = simDeviceSim.getDouble("Applied Output");
        simVelocity = simDeviceSim.getDouble("Velocity");
        simPosition = simDeviceSim.getDouble("Position");
        simCurrent = simDeviceSim.getDouble("Motor Current");
        simDeviceSim.getDouble("Stall Torque").set(gearbox.stallTorqueNewtonMeters);
        simDeviceSim.getDouble("Free Speed")
                .set(Units.radiansPerSecondToRotationsPerMinute(gearbox.freeSpeedRadPerSec));
        this.dtSeconds = dtSeconds;
    }

    public void setInputVoltage(double voltage) {
        dcMotorSim.setInputVoltage(voltage);
    }

    public void update() {
        dcMotorSim.update(dtSeconds);
        simAppliedOutput.set(dcMotorSim.getInput(0) / RobotController.getBatteryVoltage());
        simVelocity.set(dcMotorSim.getAngularVelocityRPM());
        simPosition.set(dcMotorSim.getAngularPositionRotations());
        simCurrent.set(dcMotorSim.getCurrentDrawAmps());
    }

}
