#include "mbed.h"
#include "Stepper.h"
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

Serial pc(USBTX,USBRX);
Timer MQTimer;

DigitalOut EnablePIN(D2);
Stepper MQ1(D6,D5);
Stepper MQ2(D4,D3);
Stepper MQ3(D8,D7);
Stepper MQ4(D10,D9);
Stepper MQ5(D12,D11);

//D6;D5 - q1 [clk dir]
//D4;D3 - q2 [clk dir]
//D8;D7 - q3 [clk dir]
//D10;D9 - q4 [clk dir]
//D12;D11 - q5 [clk dir]

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

const float K_SVP = 22.85;
const float K_Q_Virtual = 0.745;
const float K_Q4 = 0.57;
const int SVP_1_3_start = 220;
const int SVP_2_start = 250 + 105;
const float pi = 3.1416;
//pay attention
// SVP_1_3_start = 295
// SVP_2_start = 355
void init()
{
    MQ1.setSpeed(900);
    MQ2.setSpeed(900);
    MQ3.setSpeed(900);
    MQ4.setSpeed(900);
    MQ5.setSpeed(900);
// def 1200
    MQ1.setAcceleration(300);
    MQ2.setAcceleration(300);
    MQ3.setAcceleration(300);
    MQ4.setAcceleration(600);
    MQ5.setAcceleration(600);
// def 4000
    MQ1.setDeceleration(7000);
    MQ2.setDeceleration(7000);
    MQ3.setDeceleration(7000);
    MQ4.setDeceleration(1500);
    MQ5.setDeceleration(1500);

    MQ1.setPositionZero();
    MQ2.setPositionZero();
    MQ3.setPositionZero();
    MQ4.setPositionZero();
    MQ5.setPositionZero();
}

void moveMotor(Stepper &M, int angle)
{
    M.goesTo(angle);
    while(!M.stopped());
}

void movePairs(Stepper &M1, Stepper &M2, int A1, int A2)
{
    M1.goesTo(A1);
    M2.goesTo(A2);
    while(!M1.stopped()&&!M2.stopped());
}
void moveMotors(Stepper &M1, Stepper &M2, Stepper &M3,Stepper &M4, Stepper &M5, float A1, float A2, float A3, float A4, float A5)
{
    MQTimer.start();
    M1.goesTo(static_cast<int>(K_SVP * (K_Q_Virtual * A1 - SVP_1_3_start)));
    M2.goesTo(static_cast<int>(K_SVP * (A2 - SVP_2_start)));
    M3.goesTo(static_cast<int>(K_SVP * (K_Q_Virtual * A3 - SVP_1_3_start)));
    M4.goesTo(static_cast<int>(K_Q4 * A4));
    M5.goesTo(static_cast<int>(A5));
    while(!M1.stopped()&&!M2.stopped()&&!M3.stopped()&&!M4.stopped()&&!M5.stopped());
    MQTimer.stop();
    pc.printf("Done\n");
}

int main()
{
    EnablePIN.write(0);
    init();
    pc.baud(9200);
    pc.printf("Machine started\n");
    char charBuffer[2000];
    const char SEPARATOR = ',';
    const char CMD_SEPARATOR = ' ';
    vector<string> q_arr;
    vector<string> cmd;
    float q1,q2,q3,q4,q5;
    while (true) 
    {
        string data = pc.gets(charBuffer, 2000);
        if(data.length() > 0)
        {
            if(data.find("/correct") != string::npos)
            {
                MQ4.goesTo(-52);
                while(!MQ4.stopped());
                MQ4.setPositionZero();
                pc.printf("corrected\n");
                continue;
            }
            if(data.find("/get_time") != string::npos)
            {
                pc.printf((to_string(MQTimer.read()) + '\n').c_str());
                continue;
            }
            if(data.find("/reset_timer") != string::npos)
            {
                MQTimer.reset();
                pc.printf("timer reset\n");
                continue;
            }
            split(data, SEPARATOR, q_arr);
            if(q_arr.size() != 5) 
            {
                q_arr.clear();
                pc.printf("Not enough data\n");
                continue;
            };
            q1 = stof(q_arr[0]);
            q2 = stof(q_arr[1]);
            q3 = stof(q_arr[2]);
            q4 = stof(q_arr[3]);
            q5 = stof(q_arr[4]);
            moveMotors(MQ1, MQ2, MQ3, MQ4, MQ5, q1, q2, q3, q4, q5);
            q_arr.clear();
        }

    }
}

