#include <stdlib.h>
#include <unistd.h>
#include <ola/DmxBuffer.h>
#include <ola/Logging.h>
#include <ola/client/StreamingClient.h>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <cstdlib>
#include "NanoKontrol2.h"

using std::cout;
using std::endl;

#define true 1
#define false 0
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

//HSV 0-1.0 for all variables
struct HSV {
  float h;       // hue
  float s;       // saturation
  float v;       // brightness
};

//RGB 0-255 for all variables
struct RGB{
  unsigned int r;           // Red
  unsigned int g;           // Green
  unsigned int b;           // Blue
};


/*
 Color conversion
 */
struct HSV rgb2hsv(struct RGB* color)
{
    struct HSV out;
    float vmin, vmax, delta, rc, gc, bc;
    
    //Convert RGB 0-255 to 1.0 floats
    rc = (float)color->r / 255.0;
    gc = (float)color->g / 255.0;
    bc = (float)color->b / 255.0;
    
    //Determine the current maximum color value RGB
    vmax = MAX(rc, MAX(gc, bc));
    
    //Determine the current minimum color value RGB
    vmin = MIN(rc, MIN(gc, bc));
    
    //Compare the difference between min and max
    delta = vmax - vmin;
    
    //Difference between min and max equals color value
    out.v = vmax;
    
    if (vmax != 0.0){
        out.s = delta / vmax;
    }
    else{
        out.s = 0.0;
    }
    
    if (out.s == 0.0) {
        out.h = 0.0;
    }
    else {
        if (rc == vmax){
            out.h = (gc - bc) / delta;
        }
        else if (gc == vmax){
            out.h = 2 + (bc - rc) / delta;
        }
        else if (bc == vmax){
            out.h = 4 + (rc - gc) / delta;
        }
        out.h /= 6.0;
        if (out.h < 0){
            out.h += 1.0;
        }
    }
    return out;
}

struct RGB hsv2rgb(struct HSV hsv){
    
    // HSV scaled to the color wheel (0-255)
    float h, s, v;
    struct RGB rgb;
    
    float r = 0;
    float g = 0;
    float b = 0;
    
    // Scale Hue to be between 0 and 360. Saturation
    // and value scale to be between 0 and 1.
    // We mod it with 360 so the value is never over 1.0 (if by accident)
    h = (float) fmod((hsv.h * 360),360);
    
    s = (float) MIN(hsv.s, 1.0);
    v = (float) MIN(hsv.v, 1.0);
    
    if ( s == 0 ) {
        // If s is 0, all colors are the same.
        // This is some flavor of gray.
        r = v;
        g = v;
        b = v;
        
    }else{
        
        float p, q, t;
        float fractionalSector, sectorPos;
        int sectorNumber;
        
        // The color wheel has 6 sectors.
        // Which sector you're in.
        sectorPos = h / 60;
        sectorNumber = (int)(floor(sectorPos));
        
        // get the fractional part of the sector.
        // ie how many degrees into the sector
        fractionalSector = sectorPos - sectorNumber;
        
        // Calculate values for the three axes
        // of the color.
        p = v * (1 - s);
        q = v * (1 - (s * fractionalSector));
        t = v * (1 - (s * (1 - fractionalSector)));
        
        // Assign the fractional colors to r, g, and b
        // based on the sector the angle is in.
        switch (sectorNumber) {
            case 0:
                r = v;
                g = t;
                b = p;
                break;
                
            case 1:
                r = q;
                g = v;
                b = p;
                break;
                
            case 2:
                r = p;
                g = v;
                b = t;
                break;
                
            case 3:
                r = p;
                g = q;
                b = v;
                break;
                
            case 4:
                r = t;
                g = p;
                b = v;
                break;
                
            case 5:
                r = v;
                g = p;
                b = q;
                break;
        }
    }
    //Convert to 0-255 int form
    rgb.r = (int)(r * 255);
    rgb.g = (int)(g * 255);
    rgb.b = (int)(b * 255);
    return rgb;
}


float scale(float value, float min, float max) {
    float scaled = ((max - min) * value) + min;
    if(scaled< 0){
        scaled = scaled + 1;
    }
    return scaled;
}

float range(float value, float center_point, float width){
    return scale(value, MIN(center_point + width/2, 1.0), MAX(center_point - width/2, 0));
}

//int screen(int base, int top){
//    return 1 - (1-base)(1-top);
//}

int blend(double alpha, int src, int dest){
    int r1 = (src >> 16) & 0xFF;
    int g1 = (src >> 8) & 0xFF;
    int b1 = src & 0xFF;
    int r2 = (dest >> 16) & 0xFF;
    int g2 = (dest >> 8) & 0xFF;
    int b2 = dest & 0xFF;
    int ar, ag, ab;
    
    ar = int(alpha * double(r1 - r2) + r2);
    ag = int(alpha * double(g1 - g2) + g2);
    ab = int(alpha * double(b1 - b2) + b2);
    
    return ab | (ag << 8) | (ar << 16);
}



float center_hue = 0.40;
float hue_width = 0.24;
float brightness_overall = 1.0;
float brightness_high = 1.0;
float brightness_low = 1.0;
float hue_speed = 0.5;
float brightness_speed = 0.5;
float ordering = 0.0;

int main(int argc, char *argv[]) {
  struct HSV out;
  int j, k;
  unsigned int universe = 1;

  struct timeval tm;
  srand (time(NULL));

  NanoKontrol2 control(1);
  ola::InitLogging(ola::OLA_LOG_WARN, ola::OLA_LOG_STDERR);
  ola::DmxBuffer buffer;

  buffer.Blackout();

  ola::client::StreamingClient ola_client((ola::client::StreamingClient::Options()));

  // Setup the client, this connects to the server
  if (!ola_client.Setup()) {
    std::cerr << "Cannot connect to the OLA server" << endl;
    exit(1);
  }

  while(1){
    for(k = 0; k < 5; k++){

       brightness_overall = control.getValue(0);
       brightness_high = control.getValue(1);
       brightness_low = control.getValue(2);  

       center_hue = control.getValue(3);
       hue_width = control.getValue(4);
       ordering = control.getValue(5);
	cout<<"center hue"<<center_hue<<endl;
cout<<"hue width"<<hue_width<<endl;
       if(ordering < 0.5){
        j = k*4;
       }else{
          if(k == 0){
                  j = 0;
          }
          else if(k % 2 == 0){
                  j = (k - 1) * 4;
          }else if(k % 2 == 1){
                  j = (k + 1) * 4;
         }
       }



       gettimeofday(&tm, NULL); 

       long t = ((tm.tv_sec) * 1000 + tm.tv_usec/1000.0) + 0.5;
       //float hue = (sin(M_PI*k/5 + t*1/hue_speed * M_PI)+sin(M_PI*k/5 + t * 1/hue_speed * M_PI))/sqrt((207/128)+(33*sqrt(33)/128))/2 +0.5;
       float hue = (exp(sin(M_PI*k/5 + t*1/8700.0 * M_PI)) - (1/M_E)) * (1/(M_E-(1/M_E)));
	float bright = (exp(sin(M_PI*k/5 + t*1/6400.0 * M_PI)) - (1/M_E)) * (1/(M_E-(1/M_E)));

       out.s = 1.0;
       out.h = range(hue, center_hue, hue_width);
      
      bright = scale(bright, 1-brightness_low, brightness_high);
      
      if(brightness_overall == 0){
        out.v = 0;
      }else{
        out.v = scale(bright, 0.01, brightness_overall);
      }

      RGB output = hsv2rgb(out);
      buffer.SetChannel(0 + j, 255);
      buffer.SetChannel(1 + j, output.r);
      buffer.SetChannel(2 + j, output.g);
      buffer.SetChannel(3 + j, output.b);

      if (!ola_client.SendDmx(universe, buffer)) {
        cout << "Sending DMX to OLA failed." << endl;
        exit(1);
      }
      usleep(10000);
    }
  }
  return 0;
}
