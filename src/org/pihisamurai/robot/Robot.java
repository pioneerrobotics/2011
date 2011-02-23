/*
 * Copyright (c) 2011 Team 1076
 */

package org.pihisamurai.robot;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.camera.AxisCamera;

public class Robot extends IterativeRobot {

    /*** Configurable variables ***/

    final int DIGITAL_IO_SLOT = 4; /* Where the digital IO module is plugged */
    final int ANALOG_IO_SLOT = 1; /* Where the analog IO module is plugged */

    final int PWM_RIGHT_1 = 4; /* Front right. (River) */
    final int PWM_RIGHT_2 = 1; /* Back right. (Kaylee) */
    final int PWM_LEFT_1 = 3; /* Front left (Inara) */
    final int PWM_LEFT_2 = 2; /* Back left (Zoe) */

    final int ARM_1 = 10; /* Elbow */
    final int ARM_2 = 9; /* Wrist */
    final int ARM_3 = 8; /* Fingers */

    /* We assume that the Digital I/O for each Jaguar is plugged into the same
       slot and numbers as the PWM plugs. */

    /* Max value PWM is capable of */
    final double PWM_NOMINALSPEED = 1.0;
    /* Fraction of nominal speed we should target in autonomous mode */
    final double AUTONOMOUS_TARGETSPEED = 0.4;

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

    /* Digital input slots for drivetrain encoders */
    final int ENC_1 = 5; /* Left */

    /* Digital input slots for arm encoders */
    final int ENC_ARM_1 = 7;
    final int ENC_ARM_2 = 12;

    /* Digital input slots for initial switches */
    final int SWITCH_1 = 14; /* Elbow */

    /* Analog input slots for initial switches */
    final int SWITCH_2 = 1; /* Wrist */
    final int SWITCH_3 = 2; /* Finger 1 */
    final int SWITCH_4 = 3; /* Finger 2 */

     /* Number of encoder pulses per meter */
    final int ENC_DISTANCE_PER_PULSE = 1;

    /* Autonomous mode: how many milliseconds to lag before autocorrecting */
    final long AUTOCORRECT_LAG = -1;

    /* Wheel: Percentage deadzone */
    final double G27_DEADZONE = 0.00;

    /* Poll rate for autonomous and teleoperated threads in ms */
    final long POLL_RATE = 1;

    /* See DriverStationBuffer for camera settings */

    /******************************/

    double PWM_CURRENTSPEED = PWM_NOMINALSPEED;

    /* Go for the side position/middle right by default. Directions are from the
       perspective of the robot, not the drivers. */
    boolean middlePosition = false, middleLeft = false;
    boolean wasAutonomous = false;
    boolean wasTeleoperated = false;

    JaguarHelper jr1, jr2, jl1, jl2;
    Victor arm1, arm2, arm3;

    Drivetrain drivetrain;
    Manipulator manipulator;
    Joystick g27;
    //Joystick joystick;

    ToggleListener perspectiveToggle;

    Gyro gyro1;
    DigitalInput opt1, opt2, opt3;
    Encoder enc1, enc2;

    DriverStation driverStation;
    DriverStationBuffer buffer;

    Autonomous autonomousThread;
    Teleoperated teleoperatedThread;

    /* The middle poles on each side are slightly higher than the side pole. Use
       a digital switch on the robot that we will flip prior to the round to set
       it to middle pole mode for autonomous. */
    boolean isMiddlePole;

    public void robotInit() {
        buffer = new DriverStationBuffer();
        driverStation = DriverStation.getInstance();

        buffer.println("*** Robot Init ***");

        jr1 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_RIGHT_1);
        jr2 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_RIGHT_2);
        jl1 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_LEFT_1);
        jl2 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_LEFT_2);

        drivetrain = new Drivetrain(jr1, jr2, jl1, jl2);
        g27 = new Joystick(1);
        //joystick = new Joystick(2);

        perspectiveToggle = new ToggleListener(8);

        opt1 = new DigitalInput(DIGITAL_IO_SLOT, OPT_1);
        opt2 = new DigitalInput(DIGITAL_IO_SLOT, OPT_2);
        opt3 = new DigitalInput(DIGITAL_IO_SLOT, OPT_3);
        enc1 = new Encoder(DIGITAL_IO_SLOT, ENC_1, DIGITAL_IO_SLOT, ENC_1+1);
        //enc2 = new Encoder(DIGITAL_IO_SLOT, ENC_2);

        //enc1.setDistancePerPulse(ENC_DISTANCE_PER_PULSE);
        //enc2.setDistancePerPulse(ENC_DISTANCE_PER_PULSE);
        gyro1 = new Gyro(ANALOG_IO_SLOT, GYRO_1);
        gyro1.setSensitivity(GYRO_SENSITIVITY);

        arm1 = new Victor(DIGITAL_IO_SLOT, ARM_1);
        manipulator = new Manipulator(arm1);

        buffer.println("Init finished");
    }

    public void autonomousInit() {
        killThreads();
        buffer.println("Autonomous");
        wasAutonomous = true;
        buffer.println("AUTO: Auto thread");
        autonomousThread = new Autonomous();
        autonomousThread.start();
        buffer.println("AUTO: Init finished");

    }

    public void autonomousPeriodic() {
        //buffer.updateCamera();
    }

    public void disabledInit() {
        killThreads();
    }

    public void teleopInit() {
        killThreads();
        buffer.println("Teleoperated");
        wasTeleoperated = true;
        buffer.println("Teleop thread");
        teleoperatedThread = new Teleoperated();
        teleoperatedThread.start();
    }

    public void teleopPeriodic() {
        //buffer.updateCamera();
    }
    /* Teleop Continuous: handles arm input */
    public void teleopContinuous() {
        if(g27.getRawButton(1) && g27.getRawButton(4)) {
            manipulator.setElbowSpeed(0);
        }
        else if(g27.getRawButton(1)) {
            manipulator.setElbowSpeed(1.0);
        }
        else if(g27.getRawButton(4)) {
            manipulator.setElbowSpeed(-1.0);
        }
        else {
            manipulator.setElbowSpeed(0);
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

    /* Class for controlling the 2011 manipulator */
    private class Manipulator {

        /* Limit switches */
        AnalogChannel pot1, pot2, pot3;

        /* Victors */
        Victor arm1, arm2, arm3;

        Relay test;
        Manipulator(Victor arm1) {
            this.arm1 = arm1;

        }

        void setElbowSpeed(double speed) {
            arm1.set(speed);
        }

        void setAarmPosition(int position) {

        }

        void setWristGrab(boolean grab) {

        }

        void setFingerGrab(boolean grab) {

        }

        void setElbowPosition(int position, boolean high){

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

    private class Teleoperated implements Runnable {
        final long pollRate = POLL_RATE;
        private Thread thread;
        private boolean run = true;

        Teleoperated() {
            thread = new Thread(this);
        }

        void start() {
            thread.start();
        }

        public void run() {
            while(run && driverStation.isOperatorControl()) {
            try {

            int reverse = 1;

            /* Perspective reversal */
            if(perspectiveToggle.on) {
                reverse = -1;
            }

            if(!g27.getRawButton(5) && !g27.getRawButton(6)) {
            /* Positive throttle is brake, negative throttle is accelerate */
            /* Pedals must be on single axis mode */
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            PWM_CURRENTSPEED = 1;
            drivetrain.setRightSpeed(g27.getX()+g27.getThrottle()
                    *reverse);
            drivetrain.setLeftSpeed(-g27.getX()+g27.getThrottle()
                    *reverse);
            }
            else {
            drivetrain.setRightSpeed(0);
            drivetrain.setLeftSpeed(0);
            drivetrain.setBrakeRight(true);
            drivetrain.setBrakeLeft(true);
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
            buffer.println("Teleop thread stopped");
        }
    }

    private class Autonomous implements Runnable {

        final long pollRate = POLL_RATE;
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

    	Autonomous()
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
                if(phase == 1) {
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
                        buffer.println("AUTO: Phase 2: At T");
                    }
                }
                else if(phase == 2) {
                    /* Turn to match gyro position, score uberring */
                    /* ... */
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
            buffer.println("AUTO: Thread stopped");
        }
        }

    /* Prints a line to the User Messages box on the driver station software. */
    private class DriverStationBuffer {

        private int CAMERA_MAX_FPS = 5;
        private int CAMERA_COMPRESSION = 0;
        private AxisCamera.ResolutionT CAMERA_RES =
                AxisCamera.ResolutionT.k640x480;

        DriverStationLCD lcd;
        AxisCamera camera;
        String line2 = "", line3 = "", line4 = "", line5 = "", line6 = "";

        /* Null bytes because this library is retarded as fuck */
        final String nullBytes = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

        DriverStationBuffer() {
            lcd = DriverStationLCD.getInstance();
            //camera = AxisCamera.getInstance();

            //camera.writeResolution(AxisCamera.ResolutionT.k640x480);
            //camera.writeMaxFPS(CAMERA_MAX_FPS);
            //camera.writeCompression(CAMERA_COMPRESSION);
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

        void updateCamera() {
            lcd.updateLCD();
        }
    }

    private void killThreads() {
        if(wasAutonomous) {
            autonomousThread.stop();
        }
        if(wasTeleoperated) {
            teleoperatedThread.stop();
        }

        drivetrain.setRightSpeed(0);
        drivetrain.setLeftSpeed(0);

        wasAutonomous = false;
        wasTeleoperated = false;
    }

    private class analogSwitch {
        AnalogChannel channel;
        analogSwitch(AnalogChannel channel) {
            this.channel = channel;
        }
        boolean get() {
            return (channel.getVoltage() > 2.5);
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