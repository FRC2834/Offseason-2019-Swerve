package frc.robot;

import frc.robot.Robot;

/**
 * @author Jackson Bahm
 * Managing the network tables
 * implement this to send/recive data
 */
public interface DashboardSender{
	
	DashboardSender[] senders = {
		Robot.drivetrain
	};
	
	static void sendPeriodicData() {
		try {
			for(DashboardSender s : senders) {
				s.dashboardPeriodic();
			}
		} catch (NullPointerException e) {
			e.printStackTrace();
		}
	}
	
	static void sendInitData() {
		try {
			for(DashboardSender s : senders) {
				s.dashboardInit();
			}
		} catch (NullPointerException e) {
			e.printStackTrace();
		}
	}
	
	void dashboardInit();
	void dashboardPeriodic();
}
