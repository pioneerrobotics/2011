/*
 * Copyright (c) 2011 Team 1076
 */

package org.pihisamurai.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Victor;

public class Robot extends IterativeRobot {

    /*** Configurable variables ***/

    final int DIGITAL_IO_SLOT = 4; /* Where the digital IO module is plugged */
    final int ANALOG_IO_SLOT = 1; /* Where the analog IO module is plugged */

    final int PWM_RIGHT_1 = 4; /* Front right. (River) */
    final int PWM_RIGHT_2 = 1; /* Back right. (Kaylee) */
    final int PWM_LEFT_1 = 3; /* Front left (Inara) */
    final int PWM_LEFT_2 = 2; /* Back left (Zoe) */

    /* We assume that the Digital I/O for each Jaguar is plugged into the same
       slot and numbers as the PWM plugs. */
    
    /* Max value PWM is capable of */
    final double PWM_NOMINALSPEED = 1.0;
    /* Fraction of nominal speed we should target in autonomous mode */
    final double AUTONOMOUS_TARGETSPEED = 0.5;

    /* Gyro sensitivity */
    final double GYRO_SENSITIVITY = 0.007;

    /* Analog input slot for gyro1 */
    final int GYRO_1 = 1;

    /* Digital input slots for optical sensors */
    final int OPT_1 = 9; /* Left 9 */
    final int OPT_2 = 11; /* Right */
    final int OPT_3 = 10; /* Center */

    /* Number of meters from OPT_3 to the robot's center of rotation */
    final double OPT_OFFSET = 0.5;

    /* Digital input slots for encoders */
    final int ENC_1 = 5; /* Left */
    final int ENC_2 = 7; /* Right */

    /* Digital input slots for initial switches */
    final int SWITCH_1 = 5;
    final int SWITCH_2 = 6;

     /* Number of encoder pulses per meter */
    final int ENC_DISTANCE_PER_PULSE = 1;

    /* Autonomous mode: how many milliseconds to lag before autocorrecting */
    final long AUTOCORRECT_LAG = -1;

    /* Wheel: Percentage deadzone */
    final double G27_DEADZONE = 0.00;

    /******************************/

    double PWM_CURRENTSPEED = PWM_NOMINALSPEED;

    /* Go for the side position/middle right by default. Directions are from the
       perspective of the robot, not the drivers. */
    boolean middlePosition = false, middleLeft = false;
    boolean wasAutonomous = false;

    int reverse;

    JaguarHelper jr1, jr2, jl1, jl2;

    Drivetrain drivetrain;
    Manipulator manipulator;
    Joystick g27;
    double g27X;
    double g27Throttle;

    ToggleListener perspectiveToggle;

    Gyro gyro1;
    DigitalInput opt1, opt2, opt3;
    Encoder enc1, enc2;

    DriverStation driverStation;
    DriverStationBuffer buffer;

    AutonomousMiddle middleThread;
    AutonomousSide sideThread;

    /* The middle poles on each side are slightly higher than the side pole. Use
       a digital switch on the robot that we will flip prior to the round to set
       it to middle pole mode for autonomous. */
    boolean isMiddlePole;

    public void robotInit() {
        buffer = new DriverStationBuffer();
        driverStation = DriverStation.getInstance();

        buffer.println("*** Robot Init ***");

        drivetrain = new Drivetrain();
        g27 = new Joystick(1);

        perspectiveToggle = new ToggleListener(8);

        opt1 = new DigitalInput(DIGITAL_IO_SLOT, OPT_1);
        opt2 = new DigitalInput(DIGITAL_IO_SLOT, OPT_2);
        opt3 = new DigitalInput(DIGITAL_IO_SLOT, OPT_3);
        enc1 = new Encoder(DIGITAL_IO_SLOT, ENC_1, DIGITAL_IO_SLOT, ENC_1+1);
        //enc2 = new Encoder(DIGITAL_IO_SLOT, ENC_2);

        enc1.setDistancePerPulse(ENC_DISTANCE_PER_PULSE);
        //enc2.setDistancePerPulse(ENC_DISTANCE_PER_PULSE);
        gyro1 = new Gyro(ANALOG_IO_SLOT, GYRO_1);
        gyro1.setSensitivity(GYRO_SENSITIVITY);

        manipulator = new Manipulator();

        buffer.println("Init finished");
    }

    public void autonomousInit() {
        buffer.println("Autonomous");

        wasAutonomous = true;

        Alliance alliance = driverStation.getAlliance();

        if(alliance == DriverStation.Alliance.kBlue) {
            buffer.println("Blue alliance");
        }
        else if(alliance == DriverStation.Alliance.kRed) {
            buffer.println("Red alliance");
        }
        buffer.println("Driver pos "+driverStation.getLocation());

        /* Autonomous switches */
        //DigitalInput switch1 = new DigitalInput(DIGITAL_IO_SLOT, SWITCH_1),
                //switch2 = new DigitalInput(DIGITAL_IO_SLOT, SWITCH_2);
        //if(switch1.get()) {
        if(true) {
            middlePosition = true;
            //if(switch2.get()) {
            if(true) {
                middleLeft = true;
            }
        }

        if(middlePosition) {
            buffer.println("AUTO: Middle pos");
            if(middleLeft) {
                buffer.println("AUTO: Target left");
            }
            else {
                buffer.println("AUTO: Target right");
            }
            buffer.println("AUTO: Middle thread");
            middleThread = new AutonomousMiddle();
            middleThread.start();
        }
        else {
            buffer.println("AUTO: Side position");
            buffer.println("AUTO: Side thread");
            sideThread = new AutonomousSide();
            sideThread.start();
        }
        buffer.println("AUTO: Init finished");

    }

    public void autonomousContinuous() {

        
    }

    public void disabledInit() {
        buffer.println("Disabled");
        if(wasAutonomous) {
        /* Kill all autonomous functions */
        if(middlePosition) {
            middleThread.stop();
        }
        else {
            sideThread.stop();
        }
        }

        drivetrain.setRightSpeed(0);
        drivetrain.setLeftSpeed(0);
        
        wasAutonomous = false;
    }

    public void teleopInit() {
        buffer.println("Teleoperated");
    }

    public void teleopContinuous() {
            /* Perspective reversal */
            if(perspectiveToggle.on) {
                reverse = -1;
            }
            else {
                reverse = 1;
            }

            if(!g27.getRawButton(5) && !g27.getRawButton(6)) {
            /* Positive throttle is brake, negative throttle is accelerate */
            /* Pedals must be on single axis mode */
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            PWM_CURRENTSPEED = 1;

            g27X = g27.getX(); g27Throttle = g27.getThrottle();

            if(Math.abs(g27X) <= G27_DEADZONE) {
                g27X = 0;
            }
            if(Math.abs(g27Throttle) <= G27_DEADZONE) {
                g27Throttle = 0;
            }

            drivetrain.setRightSpeed(g27X+g27Throttle
                    *reverse);
            drivetrain.setLeftSpeed(-g27X+g27Throttle
                    *reverse);
            }
            else {
            drivetrain.setRightSpeed(0);
            drivetrain.setLeftSpeed(0);
            drivetrain.setBrakeRight(true);
            drivetrain.setBrakeLeft(true);
            }
            //printOpticalStatus();
            /*System.out.println("Center: "+opt3.get()+"Left: "+opt1.get()+
                    "Right: "+opt2.get()); */
        //printEncoderStatus();
        /* System.out.println("Gyro 1, 1: "+gyro1.getAngle() % 180 + 180); */

        //System.out.println("end1a: "+end1a.get()+" end1b: "+end1b.get()+
                //" end2a: "+end2a.get()+" end2b: "+end2b.get());
        //System.out.println("Something weird");
        /* printButtonsHeld(); */

        /*System.out.println("Joystick X: "+g27.getX());
          System.out.println("Joystick Y: "+g27.getY());
          System.out.println("Joystick Z: "+g27.getZ());
          System.out.println("Joystick T: "+g27.getThrottle());*/
            //drivetrain.printStatus();
    }

    /* Simple drivetrain class for four motor tank drive */
    private class Drivetrain {

        JaguarHelper r1, r2, l1, l2;

        boolean retard = false;

        Drivetrain() {
            r1 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_RIGHT_1);
            r2 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_RIGHT_2);
            l1= new JaguarHelper(DIGITAL_IO_SLOT, PWM_LEFT_1);
            l2 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_LEFT_2);
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
        void setRightSpeed(double speed) {
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
        void setLeftSpeed(double speed) {
           if(speed > PWM_CURRENTSPEED) {
                speed = PWM_CURRENTSPEED;
            }
            else if(speed < -PWM_CURRENTSPEED) {
                speed = -PWM_CURRENTSPEED;
            }
            l1.jaguar.set(speed);
            if(!retard) l2.jaguar.set(speed);
        }
        
        void setBrakeRight(boolean brake) {
            setRightSpeed(0);
            r1.digitalOutput.set(brake);
            r2.digitalOutput.set(brake);
        }

        void setBrakeLeft(boolean brake) {
            setLeftSpeed(0);
            l1.digitalOutput.set(brake);
            l2.digitalOutput.set(brake);
        }

        void printStatus() {
            //System.out.println("r1 "+r1.jaguar.get()+" r2 "+r2.jaguar.get()+
            //        " l1"+l1.jaguar.get()+" l2"+l2.jaguar.get());
            buffer.println("R1: "+r1.jaguar.get());
            buffer.println("R2: "+r2.jaguar.get());
            buffer.println("L1: "+l1.jaguar.get());
            buffer.println("L2: "+l2.jaguar.get());
        }
    }

    /* Class for controlling the 2011 manipulator */
    private class Manipulator {

        /* Limit switches */
        DigitalInput lim1, lim2, lim3, lim4, lim5, lim6, lim7, lim8, lim9,
                lim10;

        /* Victors */
        Victor vic1, vic2, vic3, vic4, vic5;
        
        Relay test;
        Manipulator() {
            

        }

        void setArmPosition(int position) {

        }

        void fullForward() {
            test.set(Relay.Value.kForward);
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

    /* Usage: ToggleListener tl = new ToggleListener(buttonNumber). Read boolean
       from tl.on for current toggle state.  */
    private class ToggleListener implements Runnable {

        int buttonNumber;
        long pollRate = 10;
        private Thread thread;
        private boolean run = true;
        private boolean isPressed = false;
        boolean on = false;


    	ToggleListener(int buttonNumber) {
            this.buttonNumber = buttonNumber;
            thread = new Thread(this);
            thread.start();
    	}

        ToggleListener(int buttonNumber, long pollRate) {
            this.pollRate = pollRate;
            this.buttonNumber = buttonNumber;
            thread = new Thread(this);
            thread.start();
    	}

    	public void run() {
            while(run) {
                boolean wasPressed = isPressed;

                try {
                if(g27.getRawButton(buttonNumber)) {
                    isPressed = true;
                }
                else {
                    isPressed = false;
                }

                if(wasPressed && !isPressed) {
                    on = !on;
                }

                Thread.sleep(pollRate);
                }
                catch(Exception e) {
                    System.out.println("Exception: "+e);
                }
            }    	
        }

        void stop() {
            run = false;
        }
        }

    private class AutonomousMiddle implements Runnable {

        final long pollRate = 1;
        /* Cycles we should wait before correcting direction */
        final long autoCorrectLag = AUTOCORRECT_LAG;
        long autoCorrect = 0;
        private Thread thread;
        private boolean run = true;
        DigitalInput sensor1, sensor2, sensor3;
        boolean sensor1On, sensor2On, sensor3On;
        Encoder encoder1;//, encoder2;
        Gyro gyro;
        double gyroPosition = 0;
        double targetSpeed;
        double encoderInitial;

    	AutonomousMiddle()
        {
            sensor1 = opt1;
            sensor2 = opt2;
            sensor3 = opt3;
            encoder1 = enc1;
            //encoder2 = enc2;
            gyro = gyro1;
            targetSpeed = PWM_NOMINALSPEED*AUTONOMOUS_TARGETSPEED;
            thread = new Thread(this);
        }

        /* Only works on straights! */
        double getEncoderDistance() {
             return (encoder1.getDistance());//+encoder2.getDistance())/2;
        }

        void fullForward() {
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            drivetrain.setRightSpeed(-targetSpeed);
            drivetrain.setLeftSpeed(-targetSpeed);
        }

        void turnRight() {
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            drivetrain.setRightSpeed(targetSpeed);
            drivetrain.setLeftSpeed(-targetSpeed);
        }

        void turnLeft() {
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            drivetrain.setRightSpeed(-targetSpeed);
            drivetrain.setLeftSpeed(targetSpeed);
        }

        void start() {
            run = true;
            buffer.println("AUTO: Middle thread start");
            buffer.println("AUTO: Phase 1: Start");
            thread.start();
        }

    	public void run() {

            int phase = 1;

            while(run && driverStation.isAutonomous()) {
                try {

                sensor1On = !sensor1.get();
                sensor2On = !sensor2.get();
                sensor3On = !sensor3.get();

                /* Phase 1: Robot is on middle line. Full forward. */
                /* Phase 3: Robot is on line after fork. Full forward. */
                if(phase == 1 || phase == 5) {
                    if(!sensor3On && !sensor1On && !sensor2On) {
                        fullForward();
                        autoCorrect = 0;
                    }

                    else if(sensor3On && !sensor1On && !sensor2On) {
                        fullForward();
                        autoCorrect = 0;
                        /* Take initial gyro reading */
                        if(gyroPosition != 0)
                        gyroPosition = gyro.getAngle();
                    }
                    /* Autocorrect: left sensor ON, right sensor OFF */
                    else if(sensor1On && !sensor2On) {
                        if(autoCorrect < autoCorrectLag) {
                        fullForward();
                        autoCorrect++;
                        }
                        else {
                            turnLeft();
                        }                        
                    }
                     /* Autocorrect: left sensor OFF, right sensor ON */
                    else if(!sensor1On && sensor2On) {
                        if(autoCorrect < autoCorrectLag) {
                        fullForward();
                        autoCorrect++;
                        }
                        else {
                            turnRight();
                        }
                    }
                    /* WTF situation: all three sensors on. */
                    else if(sensor3On && sensor1On && sensor2On) {
                        phase++;
                        if(phase == 2)
                        buffer.println("AUTO: Phase 2: Fork");
                        else
                        buffer.println("AUTO: Phase 5: Pre-hang");
                    }
                }
                else if(phase == 2) {
                    if(sensor3On && sensor1On && sensor2On) {
                    fullForward();
                    }
                    /* We're now at the fork. Drive forward for OPT_OFFSET and
                       turn left or right. */
                    else if(sensor3On && !sensor1On && !sensor2On) {
                        encoder1.reset();
                        //encoder2.reset();
                        encoderInitial = getEncoderDistance();

                        phase++;
                        buffer.println("AUTO: Phase 3: Fork");

                    }
                    
                }
                else if(phase == 3) {
                    fullForward();
                    if(getEncoderDistance() > OPT_OFFSET) {
                        drivetrain.setBrakeRight(true);
                        drivetrain.setBrakeLeft(true);
                        phase++;
                    }
                }
                else if(phase == 4) {
                    if(sensor1On || sensor2On) {
                            autoCorrect = autoCorrectLag;
                            phase++;
                        }

                        if(middleLeft) {
                        turnLeft();
                        }
                        else {
                        turnRight();
                        }
                }
                /* Final phase: robot has reached the final WTF situation. Turn
                   based on initial gyro reading, adjust, and score.*/
                else if(phase == 6) {
                    drivetrain.setBrakeRight(true);
                    drivetrain.setBrakeLeft(true);
                    break;
                }

                Thread.sleep(pollRate);
                }
                catch(Exception e) {
                    System.out.println("Exception: "+e);
                }
            }
        }

        void stop() {
            run = false;
        }
        }

     private class AutonomousSide implements Runnable {

        final long pollRate = 1;
        private Thread thread;
        private boolean run = true;
        DigitalInput sensor1, sensor2, sensor3;
        boolean sensor1On, sensor2On, sensor3On;
        Gyro gyro;
        double gyroPosition;

    	AutonomousSide()
        {
            thread = new Thread(this);
        }

        void start() {
            run = true;
            buffer.println("AUTO: Side thread start");
            thread.start();
        }

    	public void run() {
            while(run && driverStation.isAutonomous()) {
                try {



                Thread.sleep(pollRate);
                }
                catch(Exception e) {
                    System.out.println("Exception: "+e);
                }
            }
        }

        void stop() {
            run = false;
        }
        }


    /* Prints a line to the User Messages box on the driver station software. */
    private class DriverStationBuffer {

        DriverStationLCD lcd;
        String line2 = "", line3 = "", line4 = "", line5 = "", line6 = "";

        /* Null bytes because this library is retarded as fuck */
        final String nullBytes = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

        DriverStationBuffer() {
            lcd = DriverStationLCD.getInstance();
        }

        void println(String line) {
            System.out.println("DRIVERSTATION: "+line);

            line2 = line3;
            line3 = line4;
            line4 = line5;
            line5 = line6;
            line6 = line;

            lcd.println(Line.kUser6, 1, line6+nullBytes);
            lcd.println(Line.kUser5, 1, line5+nullBytes);
            lcd.println(Line.kUser4, 1, line4+nullBytes);
            lcd.println(Line.kUser3, 1, line3+nullBytes);
            lcd.println(Line.kUser2, 1, line2+nullBytes);

            lcd.updateLCD();
        }
    }

    /* Special/useful debug functions */
    private void printButtonsHeld() {
        for(int i = 1; i <= 12; i++) {
            boolean held = g27.getRawButton(i);
            if(held)
            System.out.println("Button "+i+": "+held);
        }
    }

    private void printEncoderStatus() {
        System.out.println("en1: "+enc1.get()+" en2: "+enc2.get());
    }

    private void printOpticalStatus() {
        System.out.println("Center: "+opt3.get()+" Left: "+opt1.get()+
            " Right: "+opt2.get());
    }
 }