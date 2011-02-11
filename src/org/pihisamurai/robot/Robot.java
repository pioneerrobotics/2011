/*
 * Copyright (c) 2011 Team 1076
 */

package org.pihisamurai.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {

    final int PWM_SLOT = 4; /* Where the PWM module is plugged */

    final int PWM_RIGHT_1 = 4; /* Front right. (River) */
    final int PWM_RIGHT_2 = 1; /* Back right. (Kaylee) */
    final int PWM_LEFT_1 = 3; /* Front left (Inara) */
    final int PWM_LEFT_2 = 2; /* Back left (Zoe) */

    /* We assume that the Digital I/O for each Jaguar is plugged into the same
       slot and numbers as the PWM plugs. */

    final double PWM_NOMINALSPEED = 1.0; /* Max value PWM is capable of */
    double PWM_CURRENTSPEED = PWM_NOMINALSPEED;

    JaguarHelper jr1;
    JaguarHelper jr2;
    JaguarHelper jl1;
    JaguarHelper jl2;

    Drivetrain drivetrain;
    Joystick joystick1;

    public void robotInit() {
        jr1 = new JaguarHelper(PWM_SLOT, PWM_RIGHT_1);
        jr2 = new JaguarHelper(PWM_SLOT, PWM_RIGHT_2);
        jl1 = new JaguarHelper(PWM_SLOT, PWM_LEFT_1);
        jl2 = new JaguarHelper(PWM_SLOT, PWM_LEFT_2);

        drivetrain = new Drivetrain(jr1, jr2, jl1, jl2);
        joystick1 = new Joystick(1);
    }

    public void autonomousPeriodic() {

    }

    public void teleopPeriodic() {

        if(!joystick1.getRawButton(1)) { /* Trigger is brake */
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
                PWM_CURRENTSPEED = PWM_NOMINALSPEED*((1-joystick1.getZ())/2);
            drivetrain.setRightSpeed(joystick1.getY()+joystick1.getX());
            drivetrain.setLeftSpeed(joystick1.getY()-joystick1.getX());
            }
        /* Braking */
        else {
            drivetrain.setRightSpeed(0);
            drivetrain.setLeftSpeed(0);
            drivetrain.setBrakeRight(true);
            drivetrain.setBrakeLeft(true);
        }
    }

    /* Simple drivetrain class for four motor tank drive */
    private class Drivetrain {

        JaguarHelper r1, r2, l1, l2;

        boolean retard = false;

        Drivetrain(JaguarHelper r1, JaguarHelper r2, JaguarHelper l1,
                JaguarHelper l2) {
            this.r1 = r1;
            this.r2 = r2;
            this.l1 = l1;
            this.l2 = l2;
            }

        /* Retard mode true = only runs one pair of motors */
        public boolean setRetardedMode(boolean set) {
            retard = set;
            if(retard) {
                r2.jaguar.set(0);
                l2.jaguar.set(0);
                return true;
            }
            return false;
        }

        /* Sets the speed of the right hand side of the drivetrain */
        public void setRightSpeed(double speed) {
            if(speed > PWM_CURRENTSPEED) {
                speed = PWM_CURRENTSPEED;
            }
            else if(speed < -PWM_CURRENTSPEED) {
                speed = -PWM_CURRENTSPEED;
            }
            r1.jaguar.set(-speed);
            if(!retard) r2.jaguar.set(-speed);
        }

        /* Sets the speed of the left hand side of the drivetrain */
        public void setLeftSpeed(double speed) {
           if(speed > PWM_CURRENTSPEED) {
                speed = PWM_CURRENTSPEED;
            }
            else if(speed < -PWM_CURRENTSPEED) {
                speed = -PWM_CURRENTSPEED;
            }
            l1.jaguar.set(speed);
            if(!retard) l2.jaguar.set(speed);
        }
        
        public void setBrakeRight(boolean brake) {
            r1.digitalOutput.set(brake);
            r2.digitalOutput.set(brake);
        }

        public void setBrakeLeft(boolean brake) {
            l1.digitalOutput.set(brake);
            l2.digitalOutput.set(brake);
        }
    }

    /* Encapsulates the PWM and digital outputs to a single Jaguar. We assume
       that a Jaguar plugged into PWM plug 1 is also connected to Digital IO
       plug 1, and so on. */
    private class JaguarHelper {

        Jaguar jaguar;
        DigitalOutput digitalOutput;

        JaguarHelper(int SLOT, int PLUG) {
            jaguar = new Jaguar(SLOT, PLUG);
            digitalOutput = new DigitalOutput(SLOT, PLUG);
            digitalOutput.disablePWM();
        }
    }

 }

