package com.simulation;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class REVElevatorSimulation {

    private final NewElevatorSim elevatorSim;

    private final SimDouble simAppliedOutput;
    private final SimDouble simVelocity;
    private final SimDouble simPosition;
    private final SimDouble simCurrent;

    public REVElevatorSimulation(
            int deviceId,
            double stallTorque,
            double freeSpeed,
            DCMotor gearbox,
            Measure<Voltage> kS,
            Measure<Voltage> kG, 
            Measure<Per<Voltage, Velocity<Distance>>> kV,
            Measure<Per<Voltage, Velocity<Velocity<Distance>>>> kA,
            double minHeightMeters, 
            double maxHeightMeters,
            double startingHeightMeters) {
        elevatorSim = new NewElevatorSim(
            kV.in(VoltsPerMeterPerSecond), 
            kA.in(VoltsPerMeterPerSecondSquared), 
            kG.in(Volts), 
            kS.in(Volts), 
            gearbox, 
            minHeightMeters, 
            maxHeightMeters, 
            startingHeightMeters);
        SimDeviceSim simDeviceSim = new SimDeviceSim(String.format("SPARK MAX [%s]", deviceId));
        simAppliedOutput = simDeviceSim.getDouble("Applied Output");
        simVelocity = simDeviceSim.getDouble("Velocity");
        simPosition = simDeviceSim.getDouble("Position");
        simCurrent = simDeviceSim.getDouble("Motor Current");
        simDeviceSim.getDouble("Stall Torque").set(stallTorque);
        simDeviceSim.getDouble("Free Speed").set(freeSpeed);
    }

    public void setInputVoltage(double voltage){
        elevatorSim.setInputVoltage(voltage);
    }

    public void update() {
        elevatorSim.update(0.020);
        simAppliedOutput.set(elevatorSim.getInput(0) / RobotController.getBatteryVoltage());
        simVelocity.set(elevatorSim.getVelocityMetersPerSecond());
        simPosition.set(elevatorSim.getPositionMeters());
        simCurrent.set(elevatorSim.getCurrentDrawAmps());
    }

}
