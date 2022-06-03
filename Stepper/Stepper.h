#ifndef STEPPER_H
#define STEPPER_H

#include "mbed.h"
/** Stepper class.
 *  Used for drive Stepper motor
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "Stepper.h"
 *
 * DigitalIn home(p20);
 * Stepper mot(p21,p22);
 *
 * int main()
 * {
 *    mot.setSpeed(1200);
 *    mot.setAcceleration(4000);
 *    mot.setDeceleration(4000);
 *
 *    mot.rotate(CW);
 *    while(!home);
 *    mot.stop();
 *    mot.setPositionZero();
 *
 *    mot.goesTo(1600);
 *    while(!mot.stopped());
 *
 *    while(1)
 *    {
 *
 *    }
 * }
 * @endcode
 */
class Stepper : public TimerEvent
{
public: 
    /** Create Stepper instance connected to pin clk & dir
    * @param clk pin to connect at clk/step output
    * @param dir pin to connect at dir output
    */
    Stepper(PinName clk, PinName dir);
    
    /** Moving the motor relative to a reference position (set by setPositionZero())
    * @param position steps from position zero 
    */
    void goesTo(int position); 
    
    /** Moving the motor for given steps
    * @param steps number of steps to go(>0 CW ; <0 CCW)
    */
    void move(int steps);
    
    /** Moving the motor until user stop
    * @param direction rotation Stepper::CC or Stepper::CCW
    */
    void rotate(bool direction);
    
    /** Stop the motor as fast as possible
    */
    void stop(void);
    
    /**Set Rotation Speed
    * @param speed [steps/sec]
    */
    void setSpeed(float speed);
    
    /**Set Acceleration ramp
    * @param acc acceleration [steps/sec²] (0 = infinite acceleration 1st step @ max speed)
    */
    void setAcceleration(float acc);
    
    /**Set Deceleration ramp
    * @param dec deceleration [steps/sec²] (0 = infinite deceleration last step @ max speed)
    */
    void setDeceleration(float dec);
    
    /**Give Origin for motor absolute movement
    */
    void setPositionZero(void);
    
    /**Check if motor is stopped
    * @returns true if motor stopped
    */  
    bool stopped(void);
    
    /**Get absolute position from origin
    * @returns position [steps] from origin set by setPositionZero()
    */
    int getPosition(void);
    
    /**Get Acceleration
    * @returns acceleration [steps/sec²]
    */
    float getAcceleration(void);
    
    /**Get Deceleration
    * @returns deceleration [steps/sec²]
    */
    float getDeceleration(void);
    
    /**Get Speed
    * @returns speed [steps/sec]
    */
    float getSpeed(void);
    
    /**Enum for direction
    *   CW=true or 1; CCW=false or 0
    */
    typedef enum {CW=1,CCW=0} direction;
    
protected:
    unsigned int nTo(float speed,float acc);
     
private:
    float _acc;                             //Acceleration [step/s²]
    float _dec;                             //Decceleration [step/s²]
    float _spd;                             //Speed [step/s]
    unsigned int _steps;                    //nbr total of steps per mov
    DigitalOut _clk;                        //output clock pin 
    DigitalOut _dir;                        //output dir pin
    enum {STOP,ACCEL,CRUISE,DECEL}_state;  //Motor state
    unsigned int _dt0;                      //initial delay [µs]
    unsigned int _dtmin;                    //delay minimum [µs]
    unsigned int _dtn;                             //current delay
    int _pos;                               //motor position
    unsigned int _n;                        //steps counters
    unsigned int _nStartDec;                //steps to decelerate
    virtual void handler();

};

#endif