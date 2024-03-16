/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */
#include "ros/ros.h"
#include "service_sensori/getTandH.h"
#include "service_sensori/getTandHResponse.h"
#include "service_sensori/getTandHRequest.h"
#include <unistd.h>
#include <iostream>
#include <cstddef>
#include <bitset>
#include  "serialib.h"
#include <string>
#include <sstream>


uint8_t buffer;


using  namespace std;



class thp
{
  protected:
    //ros::NodeHandle nh_;
    serialib ser;
    const unsigned char  ReadVoltage[8] = {0x01, 0x04, 0x00, 0x01, 0x00, 0x01, 0x60, 0x0A};
    const unsigned char  ReadHum[8] = {0x01, 0x04, 0x00, 0x02, 0x00, 0x01, 0x90, 0x0A}; //02 03 01 39 00 02 15 C9
    const unsigned char  th[8] = {0x01,0x04,0x00,0x01,0x00, 0x02,0x20,0x0B}; //02 03 A0 00 00 0A E7 FE
    const unsigned char  sol[8] = {0x05,0x03,0x00,0x00,0x00, 0x01,0x85,0x8E}; //05 03 00 00 00 01 85 8E
  public:
    float temperatura;
    float umidita;
    float   radiazione;



  ~thp(void)
  {
      ser.closeDevice();
  }

    int init()
    {
    
            int stato = ser.openDevice("/dev/ttyUSB0", 9600,
                          SERIAL_DATABITS_8,
                          SERIAL_PARITY_EVEN,
                          SERIAL_STOPBITS_1);
            if (stato !=1 )
            {
                cout << "Errore apertura Porta" << endl;
                return -1;
            }  else
            cout << "Connesso" << endl;
       return 1;
    }
    void getTh()
    {
       char buffer[26];

       ser.writeBytes(th,sizeof(th));
       int letti = ser.readBytes (buffer,25,1000, 1000);
       if (letti == 0) 
		{
			 this->umidita = 0.0;
        		 this->temperatura = 0.0;
			return; // tensione nulla
		}
       std::stringstream ss;
       ss << "0x";
       for (int j = 0; j < 25; j++)
        {
        ss <<  std::hex << int(buffer[j] & 255);
        }
      
       int th = 0;
       for (int kk = 3; kk < 5;kk++) {
                    th = (th << 8);
                    int high = ((buffer[kk] & 0xF0 )>> 4);
                    int low = int(buffer[kk] & 0x0f);
                    th += int((high*16+low) );

                }
        
        int hmd = 0;
        for (int kk = 5; kk < 7;kk++) {
                    hmd = (hmd << 8);
                    int high = ((buffer[kk] & 0xF0 )>> 4);
                    int low = int(buffer[kk] & 0x0f);
                    hmd += int((high*16+low) );

                }
        this->umidita = hmd/100.00;
        this->temperatura = th/100.0;
    };

float getHum()
    {
       char buffer[10];

       ser.writeBytes(ReadHum,sizeof(ReadHum));
       int letti = ser.readBytes (buffer,9,1000, 1000);
       if (letti == 0) return 0.0; // tensione nulla
  //     cout << "letti: " << letti << endl;
       std::stringstream ss;
       ss << "0x";
              for (int j = 0; j < 9; j++)
        {
        ss <<  std::hex << int(buffer[j] & 255);
        }
   //    cout << ss.str() << endl;
        int current = 0;
       for (int kk = 3; kk < 5;kk++) {
                    current = (current << 8);
                    int high = ((buffer[kk] & 0xF0 )>> 4);
                    int low = int(buffer[kk] & 0x0f);
                    current += int((high*16+low) );

                }
        return (float)(current / 100.00);
    };
    float getTemp()
        {
       char buffer[8];
       ser.writeBytes(ReadVoltage,sizeof(ReadVoltage));
       int letti = ser.readBytes (buffer,7,1000, 1000);
       if (letti == 0) return 0.0; // tensione nulla
       std::stringstream ss;
            ss << "0x";
       for (int j = 0; j < 7; j++)
        {
        ss <<  std::hex << int(buffer[j] & 255);
        }
       
       int volt = 0;
       for (int kk = 3; kk < 5;kk++) {
                    volt = (volt << 8);
                    int high = ((buffer[kk] & 0xF0 )>> 4);
                    int low = int(buffer[kk] & 0x0f);
                    volt += int((high*16+low) );

                }
        return (float)(volt/100.00);
    }
void getSol()
        {
       char buffer[8];
       ser.writeBytes(sol,sizeof(sol));
       int letti = ser.readBytes (buffer,7,1000, 1000);
       if (letti == 0) return; // tensione nulla
       
       std::stringstream ss;
            ss << "0x";
       for (int j = 0; j < 7; j++)
        {
        ss <<  std::hex << int(buffer[j] & 255);
        }
      
       int solR = 0;
       for (int kk = 3; kk < 5;kk++) {
                    solR = (solR << 8);
                    int high = ((buffer[kk] & 0xF0 )>> 4);
                    int low = int(buffer[kk] & 0x0f);
                    solR += int((high*16+low) );

                }
        this->radiazione =  (float)solR;
    }

	bool handle_getTempAndHumidity(service_sensori::getTandH::Request &req,service_sensori::getTandH::Response &res)
	{
  		this->getTh();
    		this->getSol();
                res.temperatura = this->temperatura;
		res.umidita = this->umidita;
                res.radiazione = (int)this->radiazione;
  		return true;
	}

};





int main (int argc, char** argv){

    ros::init(argc, argv, "Service_Sensori");
    ros::NodeHandle n;
    thp node01;
    ros::ServiceServer service = n.advertiseService("getSensorData", &thp::handle_getTempAndHumidity,&node01);
    ROS_INFO("Ready to get sensor data.");
  
    node01.init();
    ros::spin();
    return 0;
}

