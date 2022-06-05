#include "mbed.h"
#include "Stepper.h"
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

DigitalOut EnablePIN(D2);
Serial pc(USBTX,USBRX);
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
const int SVP_1_3_start = 220;
const int SVP_2_start = 250 + 105;
//pay attention
// SVP_1_3_start = 314
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
void moveMotors(Stepper &M1, Stepper &M2, Stepper &M3,Stepper &M4, Stepper &M5, int A1, int A2, int A3, int A4, int A5)
{
    M1.goesTo(A1);
    M2.goesTo(A2);
    M3.goesTo(A3);
    M4.goesTo(A4);
    M5.goesTo(A5);
    while(!M1.stopped()&&!M2.stopped()&&!M3.stopped()&&!M4.stopped()&&!M5.stopped());
}

int main()
{
    EnablePIN.write(0);
    init();
    pc.baud(9200);
    pc.printf("Serial Port Started\n");
    char charBuffer[2000];
    const char SEPARATOR = ',';
    vector<string> q_arr;
    const float pi = 3.1416;
    int q1,q2,q3,q4,q5;
    while (true) 
    {
        string data = pc.gets(charBuffer, 2000);
        if(data.length() > 0)
        {
            split(data, SEPARATOR, q_arr);
            if(q_arr.size() != 5) 
            {
                q_arr.clear();
                pc.printf("Error\n");
                data = "";
                continue;
            };
            q1 = static_cast<int>(K_Q_Virtual * stof(q_arr[0]));
            q2 = static_cast<int>(stof(q_arr[1]));
            q3 = static_cast<int>(K_Q_Virtual * stof(q_arr[2]));
            q4 = static_cast<int>(stof(q_arr[3]));
            q5 = static_cast<int>(stof(q_arr[4]));
            moveMotors(MQ1, MQ2, MQ3, MQ4, MQ5, K_SVP * (q1 - SVP_1_3_start), K_SVP * (q2 - SVP_2_start), K_SVP * (q3 - SVP_1_3_start), q4, q5);
            data = "";
            q_arr.clear();
            pc.printf("Done\n");
        }

    }
}

