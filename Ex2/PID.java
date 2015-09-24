package Ex2;

import lejos.nxt.Button;

import ch.aplu.nxt.LightSensor;
import ch.aplu.nxt.Motor;
import ch.aplu.nxt.MotorPort;
import ch.aplu.nxt.NxtRobot;
import ch.aplu.nxt.SensorPort;

public class PID {

	LightSensor ls;
	NxtRobot robot;
	int vWhite = -1;
	int vBlack = -1;
	int lightValue;
	double offset;
	double error;
	double lastError = 0;
	int Tp = 25;
	double Kp;
	double KpOrige;
	double KI = 0.1;
	int Kd = 2;
	double derivative;
	double Turn;
	Motor motA;
	Motor motB;
	int SpeedA;
	int SpeedB;
	double integral = 0;

	public PID() {
		robot = new NxtRobot();

		motA = new Motor(MotorPort.A);
		motB = new Motor(MotorPort.B);
		robot.addPart(motA);
		robot.addPart(motB);

		ls = new LightSensor(SensorPort.S1);
		robot.addPart(ls);
		ls.activate(true);
	}

	public static void main(String[] args) {
		PID LS = new PID();
		LS.calibrate();
		LS.goLine();

	}

	public void goLine() {
		motA.setSpeed(Tp);
		motB.setSpeed(Tp);
		motA.forward();
		motB.forward();
		long time = System.currentTimeMillis();
		long delta = 0;
		long dT = 0;
		long currentTime = System.currentTimeMillis();

		while (true) {
			dT = System.currentTimeMillis() - currentTime;
			currentTime = System.currentTimeMillis();
			lightValue = ls.getValue();
			error = lightValue - offset;
			integral = 2 / 3 * integral + error;
			derivative = error - lastError;
			Turn = Kp * error + KI * integral + Kd * derivative;
			lastError = error;

			if (error > 0) {
				delta += System.currentTimeMillis() - time;
				time = System.currentTimeMillis();
			} else {
				delta = 0;
			}

			System.out.println(delta);
			
			if (delta < 200 && delta > 100) {
				
				Kp = KpOrige/2;
			}
			else {
				Kp = KpOrige;
			}

			if (delta > 2000) {
				motA.stop();
				motB.stop();
				break;
			}
			SpeedA = (int) (Tp + Turn);

			if (SpeedA > 0) {

				motA.setSpeed(SpeedA);
				motA.forward();
			} else {
				motA.setSpeed(Math.abs(SpeedA));
				motA.backward();
			}

			SpeedB = (int) (Tp - Turn);
			if (SpeedB > 0) {
				motB.setSpeed(SpeedB);
				motB.forward();
			} else {
				motB.setSpeed(Math.abs(SpeedB));
				motB.backward();
			}

		}
	}

	public void calibrate() {
		Button.LEFT.waitForPress();
		vWhite = ls.getValue();
		System.out.println(vWhite);

		Button.RIGHT.waitForPress();
		vBlack = ls.getValue();
		System.out.println(vBlack);

		offset = (vBlack + vWhite) / 2;
		Kp = Tp / (offset - vBlack) / 3;
		KpOrige = Kp;
		System.out.println(Kp);
		Button.ENTER.waitForPress();

	}

}
