package com.simulation;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class REVMotorSimulation {

    private final NewDCMotorSim dcMotorSim;

    private final SimDouble simAppliedOutput;
    private final SimDouble simVelocity;
    private final SimDouble simPosition;
    private final SimDouble simCurrent;

    public REVMotorSimulation(
            int deviceId,
            double stallTorque,
            double freeSpeed,
            LinearSystem<N2, N1, N2> plant,
            DCMotor gearbox,
            double gearing,
            Measure<Voltage> kS) {
        dcMotorSim = new NewDCMotorSim(plant, gearbox, gearing, kS.in(Volts));
        SimDeviceSim simDeviceSim = new SimDeviceSim(String.format("SPARK MAX [%s]", deviceId));
        simAppliedOutput = simDeviceSim.getDouble("Applied Output");
        simVelocity = simDeviceSim.getDouble("Velocity");
        simPosition = simDeviceSim.getDouble("Position");
        simCurrent = simDeviceSim.getDouble("Motor Current");
        simDeviceSim.getDouble("Stall Torque").set(stallTorque);
        simDeviceSim.getDouble("Free Speed").set(freeSpeed);
    }

    public void setInputVoltage(double voltage){
        dcMotorSim.setInputVoltage(voltage);
    }

    public void update() {
        dcMotorSim.update(0.020);
        simAppliedOutput.set(dcMotorSim.getInput(0) / RobotController.getBatteryVoltage());
        simVelocity.set(dcMotorSim.getAngularVelocityRPM());
        simPosition.set(dcMotorSim.getAngularPositionRotations());
        simCurrent.set(dcMotorSim.getCurrentDrawAmps());
    }

}
