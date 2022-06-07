#include "mbed.h"
#include "Stepper.h"
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

Serial pc(USBTX, USBRX);
Timer MQTimer;

DigitalOut EnablePIN(D2);
Stepper MQ1(D6, D5);
Stepper MQ2(D4, D3);
Stepper MQ3(D8, D7);
Stepper MQ4(D10, D9);
Stepper MQ5(D12, D11); //half step




// D6;D5 - q1 [clk dir]
// D4;D3 - q2 [clk dir]
// D8;D7 - q3 [clk dir]
// D10;D9 - q4 [clk dir]
// D12;D11 - q5 [clk dir]

float saturation(float value ,float min, float max)
{
    if(value > max)
    {
        return max;
    } 
    else if (value < min) 
    {
        return min;
    }
    else
    {
        return value;
    }
}

void split(std::string const &str, const char delim,
					 std::vector<std::string> &out)
{
	size_t start;
	size_t end = 0;

	while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
	{
		end = str.find(delim, start);
		out.push_back(str.substr(start, end - start));
	}
}

//**************Semaphores**************//
bool isModeSetPos = false;      // Position configuration
bool isModeSetVel = false;		// Velosity configuration
bool isModeSetAcc = false;		// Acceleration configuration
bool isModeSetDec = false;		// Deceleration configuration
//**************Semaphores**************//
const float K_Q5_half_step_vel = 2.845;
const float K_SVP = 22.85;
const float K_Q_Virtual = 0.745;
const float K_Q4 = 0.57;
const int SVP_1_3_start = 220;
const int SVP_2_start = 250 + 105;
// pay attention
//  SVP_1_3_start = 295
//  SVP_2_start = 355
void moveMotor(Stepper &M, int angle)
{
	M.goesTo(angle);
	while (!M.stopped())
		;
}

void movePairs(Stepper &M1, Stepper &M2, int A1, int A2)
{
	M1.goesTo(A1);
	M2.goesTo(A2);
	while (!M1.stopped() && !M2.stopped())
		;
}
void moveMotors(Stepper &M1, Stepper &M2, Stepper &M3, Stepper &M4, Stepper &M5, float A1, float A2, float A3, float A4, float A5)
{
	MQTimer.reset();
	MQTimer.start();
    
    A1 = saturation(A1, 160, 560);
    A2 = saturation(A2, 155, 475);
    A3 = saturation(A3, 160, 560);
    A4 = saturation(A4, 0, 180);
    A5 = saturation(A5, 0, 180);

	M1.goesTo(static_cast<int>(K_SVP * (K_Q_Virtual * A1 - SVP_1_3_start)));
	M2.goesTo(static_cast<int>(K_SVP * (A2 - SVP_2_start)));
	M3.goesTo(static_cast<int>(K_SVP * (K_Q_Virtual * A3 - SVP_1_3_start)));
	M4.goesTo(static_cast<int>(K_Q4 * A4));
	M5.goesTo(static_cast<int>(-A5));
	do
	{
		do
		{
			do
			{
				do
				{
					do
					{
					} while (!MQ5.stopped());
				} while (!MQ4.stopped());
			} while (!MQ3.stopped());
		} while (!MQ2.stopped());
	} while (!MQ1.stopped());

	MQTimer.stop();
	pc.printf("position changed\n");
}

void basicInit()
{
	MQ1.setSpeed(900);
	MQ2.setSpeed(900);
	MQ3.setSpeed(900);
	MQ4.setSpeed(50);
	MQ5.setSpeed(50);
	// def 1200
	MQ1.setAcceleration(300);
	MQ2.setAcceleration(300);
	MQ3.setAcceleration(300);
	MQ4.setAcceleration(300);
	MQ5.setAcceleration(1200);
	// def 4000
	MQ1.setDeceleration(8000);
	MQ2.setDeceleration(8000);
	MQ3.setDeceleration(8000);
	MQ4.setDeceleration(8000);
	MQ5.setDeceleration(8000);
    // rec 8000
	MQ1.setPositionZero();
	MQ2.setPositionZero();
	MQ3.setPositionZero();
	MQ4.setPositionZero();
	MQ5.setPositionZero();
}

int main()
{
	EnablePIN.write(1);
	basicInit();
	pc.baud(9200);
	pc.printf("Machine started\n");
	char charBuffer[2000];
	const char SEPARATOR = ',';
	vector<string> q_arr;
	vector<string> vel_arr;
	vector<string> acc_arr;
	vector<string> dec_arr;
	float q1, q2, q3, q4, q5;
    string data = "";
	while (true)
	{
		q_arr.clear();
		vel_arr.clear();
		acc_arr.clear();
		dec_arr.clear();
		data = pc.gets(charBuffer, 2000);
		if (data.length() > 0)
		{
			//************************SEMAPHORES************************//
			if (isModeSetPos)
			{
				split(data, SEPARATOR, q_arr);
				if (q_arr.size() != 5)
				{
					pc.printf("Not enough data\n");
					continue;
				};
				q1 = stof(q_arr[0]);
				q2 = stof(q_arr[1]);
				q3 = stof(q_arr[2]);
				q4 = stof(q_arr[3]);
                // half step
				q5 = 1.5 * stof(q_arr[4]);
                // half step
				pc.printf("position set\n");
				isModeSetPos = false;
				continue;
			}
			if (isModeSetVel)
			{
				split(data, SEPARATOR, vel_arr);
				if (vel_arr.size() != 5)
				{
					pc.printf("Not enough data\n");
					continue;
				};
				MQ1.setSpeed(saturation(stof(vel_arr[0]), 0, 1200));
				MQ2.setSpeed(saturation(stof(vel_arr[1]), 0, 1200));
				MQ3.setSpeed(saturation(stof(vel_arr[2]), 0, 1200));
				MQ4.setSpeed(saturation(stof(vel_arr[3]), 0, 1200));
				MQ5.setSpeed(K_Q5_half_step_vel * saturation(stof(vel_arr[4]), 0, 1200));
				pc.printf("velocity set\n");
				isModeSetVel = false;
				continue;
			}
			if (isModeSetAcc)
			{
				split(data, SEPARATOR, acc_arr);
				if (acc_arr.size() != 5)
				{
					pc.printf("Not enough data\n");
					continue;
				};
				MQ1.setAcceleration(stof(acc_arr[0]));
				MQ2.setAcceleration(stof(acc_arr[1]));
				MQ3.setAcceleration(stof(acc_arr[2]));
				MQ4.setAcceleration(stof(acc_arr[3]));
                // half step
				MQ5.setAcceleration(2 * stof(acc_arr[4]));
                // half step
				pc.printf("acceleration set\n");
				isModeSetAcc = false;
				continue;
			}
			if (isModeSetDec)
			{
				split(data, SEPARATOR, dec_arr);
				if (dec_arr.size() != 5)
				{
					pc.printf("Not enough data\n");
					continue;
				};
				MQ1.setDeceleration(stof(dec_arr[0]));
				MQ2.setDeceleration(stof(dec_arr[1]));
				MQ3.setDeceleration(stof(dec_arr[2]));
				MQ4.setDeceleration(stof(dec_arr[3]));
				MQ5.setDeceleration(stof(dec_arr[4]));
				pc.printf("deceleration set\n");
				isModeSetDec = false;
				continue;
			}
			//************************SEMAPHORES************************//
			//************************CONFIGMODE************************//
			if (data.find("/set_pos") != string::npos)
			{
				isModeSetPos = true;
				isModeSetVel = false;
				isModeSetAcc = false;
				isModeSetDec = false;
				pc.printf("configure position mode is enabled\n");
				continue;
			}
			if (data.find("/set_vel") != string::npos)
			{
				isModeSetPos = false;
				isModeSetVel = true;
				isModeSetAcc = false;
				isModeSetDec = false;
				pc.printf("configure velocity mode is enabled\n");
				continue;
			}
			if (data.find("/set_acc") != string::npos)
			{
				isModeSetPos = false;
				isModeSetVel = false;
				isModeSetAcc = true;
				isModeSetDec = false;
				pc.printf("configure acceleration mode is enabled\n");
				continue;
			}
			if (data.find("/set_dec") != string::npos)
			{
				isModeSetPos = false;
				isModeSetVel = false;
				isModeSetAcc = false;
				isModeSetDec = true;
				pc.printf("configure deceleration mode is enabled\n");
				continue;
			}
			//************************CONFIGMODE************************//
			//************************COMMANDS************************//
            if(data.find("/toggle_power") != string::npos)
            {
                EnablePIN = !EnablePIN;
                (EnablePIN) ? pc.printf("Power off\n") : pc.printf("Power on\n");
                continue;
            }
			if (data.find("/move") != string::npos)
			{
				moveMotors(MQ1, MQ2, MQ3, MQ4, MQ5, q1, q2, q3, q4, q5);
				continue;
			}
			if (data.find("/get_home") != string::npos)
			{
				moveMotors(MQ1, MQ2, MQ3, MQ4, MQ5, 295, 355, 295, 0, 0);
				pc.printf("home position\n");
				continue;
			}
			if (data.find("/correct") != string::npos)
			{
				MQ4.goesTo(-52);
				while (!MQ4.stopped())
					;
				MQ4.setPositionZero();
				pc.printf("corrected\n");
				continue;
			}
            if (data.find("/uncorrect") != string::npos)
			{
				MQ4.goesTo(52);
				while (!MQ4.stopped())
					;
				MQ4.setPositionZero();
				pc.printf("corrected\n");
				continue;
			}
			if (data.find("/get_timer") != string::npos)
			{
				pc.printf("%s", (to_string(MQTimer.read()) + '\n').c_str());
				continue;
			}
			if (data.find("/reset_timer") != string::npos)
			{
				MQTimer.reset();
				pc.printf("timer reset\n");
				continue;
			}
			//************************COMMANDS************************//
            data = "";
		}
	}
}
