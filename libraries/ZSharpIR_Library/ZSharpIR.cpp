/*
	ZSharpIR

	Arduino library for retrieving distance (in mm) from the analog GP2Y0A21Y and GP2Y0A02YK,...

    Original comment from Dr. Marcal Casas-Cartagena :
	inspired from :
	- https://github.com/qub1750ul/Arduino_SharpIR.git
    - https://github.com/jeroendoggen/Arduino-GP2Y0A21YK-library.git
    - https://github.com/guillaume-rico/SharpIR.git
    - https://github.com/nikv96/MDP-Arduino.git
    - https://github.com/jeroendoggen/Arduino-GP2Y0A21YK-library.git

  

*/


#include "Arduino.h"
#include "Math.h"
#include "ZSharpIR.h"

// Initialisation function
//  + irPin : is obviously the pin where the IR sensor is attached
//  + sensorModel is a int to differentiate the two sensor models this library currently supports:
//     1080 is the int for the GP2Y0A21Y and 
//     20150 is the int for GP2Y0A02YK and 
//     100500 is the long for GP2Y0A710K0F
//    The numbers reflect the distance range they are designed for (in cm)
ZSharpIR::ZSharpIR(int irPin, const uint32_t  sensorModel) {
  
    _irPin=irPin;
    _model=sensorModel;
    
    // Define pin as Input
    pinMode (_irPin, INPUT);
    _Adcres=10;
	_refVoltage=5000;
}

// Sort an array
void ZSharpIR::sort(int a[], int size) {
    for(int i=0; i<(size-1); i++) {
        bool flag = true;
        for(int o=0; o<(size-(i+1)); o++) {
            if(a[o] > a[o+1]) {
                int t = a[o];
                a[o] = a[o+1];
                a[o+1] = t;
                flag = false;
            }
        }
        if (flag) break;
    }
}

int ZSharpIR::analogOutput(){
    int ir_val[NB_SAMPLE];

    for (int i=0; i<NB_SAMPLE; i++){
        // Read analog value
        ir_val[i] = analogRead(_irPin);
    }
    
    // Sort it 
    sort(ir_val,NB_SAMPLE);

    return ir_val[NB_SAMPLE / 2];
}


// Read distance and compute it
float ZSharpIR::distance() {

    int ir_val[NB_SAMPLE];
    float distanceMM;
    float current;


    for (int i=0; i<NB_SAMPLE; i++){
        // Read analog value
        ir_val[i] = analogRead(_irPin);
    }
    
    // Sort it 
    sort(ir_val,NB_SAMPLE);

    
    if (_model==A0)//LB
		{
        //modified 1080 for GP2Y0A21YK instead

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        //puntualDistance = 27.728 * pow(voltFromRaw / 1000., -1.2045);
        //distanceMM =( 27.728 * pow(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.2045));
        distanceMM = 9101.5*pow(ir_val[NB_SAMPLE / 2],-1.115);

    } 
    else if (_model==A3)//LF
		{
        //modified 1080 for GP2Y0A21YK instead

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        //puntualDistance = 27.728 * pow(voltFromRaw / 1000., -1.2045);
        //distanceMM =( 27.728 * pow(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.2045));
        distanceMM = 13730*pow(ir_val[NB_SAMPLE / 2],-1.159);
    } 
    else if (_model==A5)//FL
		{
        //modified 1080 for GP2Y0A21YK instead

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        //puntualDistance = 27.728 * pow(voltFromRaw / 1000., -1.2045);
        //distanceMM =( 27.728 * pow(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.2045));
        distanceMM = 24581*pow(ir_val[NB_SAMPLE / 2],-1.281);
    }     

    else if (_model==A4)//FM
		{
        //modified 1080 for GP2Y0A21YK instead

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        //puntualDistance = 27.728 * pow(voltFromRaw / 1000., -1.2045);
        //distanceMM =( 27.728 * pow(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.2045))+1.5;
        distanceMM = 25162*pow(ir_val[NB_SAMPLE/2],-1.279);
    } 
    else if (_model==A2)//FR
		{
        //modified 1080 for GP2Y0A21YK instead

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        //puntualDistance = 27.728 * pow(voltFromRaw / 1000., -1.2045);
        //distanceMM =( 27.728 * pow(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.2045));
        //taken at 6.1V
        distanceMM = 43778*pow(ir_val[NB_SAMPLE / 2],-1.371);    
    } 
        
    else if (_model==A1) //RB
	{
        //edited for GP2Y0A02YK instead

        // Previous formula used by  Dr. Marcal Casas-Cartagena
        // puntualDistance=61.573*pow(voltFromRaw/1000, -1.1068);
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        //distanceMM =( 61.573 * pow(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.1068));
        distanceMM = (36085*pow(ir_val[NB_SAMPLE/2],-1.193));
    } 

    return distanceMM;
}




/// <summary>
/// setARefVoltage:set the ADC reference voltage: (default value: 5000mV, set to 3300mV, typically 3.3 on Arduino boards)
/// </summary>
void ZSharpIR::setARefVoltage(int refV)
{

	_refVoltage=refV;
}
/// <summary>
/// SetAnalogReadResolution:set the ADC resolution : (default value: 10, set to 12, typically 10 on Arduino boards)
/// </summary>
void ZSharpIR::SetAnalogReadResolution(int res)
{

	_Adcres=res;
	//analogReadResolution( res);
}