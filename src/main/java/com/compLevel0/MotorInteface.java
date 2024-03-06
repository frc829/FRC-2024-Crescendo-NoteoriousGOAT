package com.compLevel0;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface MotorInteface {

        public Measure<Voltage> getVoltage();

        public Measure<Angle> getAngle();

        public Measure<Angle> getAbsoluteAngle();

        public Measure<Velocity<Angle>> getAngularVelocity();

        public void setRelativeEncoderAngle(Measure<Angle> angle);

        public void turn(Measure<Angle> turn);

        public void spin(Measure<Velocity<Angle>> spin);

        public void setVoltage(Measure<Voltage> voltage);

        public void stop();

        public void update();

        public static final class REV {
                public static final MotorInteface createNEO(CANSparkBase canSparkBase){
                        return new MotorInteface() {

                                @Override
                                public Measure<Voltage> getVoltage() {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'getVoltage'");
                                }

                                @Override
                                public Measure<Angle> getAngle() {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'getAngle'");
                                }

                                @Override
                                public Measure<Angle> getAbsoluteAngle() {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'getAbsoluteAngle'");
                                }

                                @Override
                                public Measure<Velocity<Angle>> getAngularVelocity() {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'getAngularVelocity'");
                                }

                                @Override
                                public void setRelativeEncoderAngle(Measure<Angle> angle) {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'setRelativeEncoderAngle'");
                                }

                                @Override
                                public void turn(Measure<Angle> turn) {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'turn'");
                                }

                                @Override
                                public void spin(Measure<Velocity<Angle>> spin) {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'spin'");
                                }

                                @Override
                                public void setVoltage(Measure<Voltage> voltage) {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
                                }

                                @Override
                                public void stop() {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'stop'");
                                }

                                @Override
                                public void update() {
                                        // TODO Auto-generated method stub
                                        throw new UnsupportedOperationException("Unimplemented method 'update'");
                                }
                                
                        };
                }
        }

}
