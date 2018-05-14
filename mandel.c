
#include "lcd/lcd.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SWC _BV(PE7)
#define SWW _BV(PC5)
#define SWS _BV(PC4)
#define SWE _BV(PC3)
#define SWN _BV(PC2)

#define CENTER_PRESSED   ((PINE & SWC) == 0)
#define CENTER_UNPRESSED ((PINE & SWC) == SWC)

#define WEST_PRESSED     ((PINC & SWW) == 0)
#define WEST_UNPRESSED   ((PINC & SWW) == SWW)

#define SOUTH_PRESSED    ((PINC & SWS) == 0)
#define SOUTH_UNPRESSED  ((PINC & SWS) == SWS)


#define EAST_PRESSED     ((PINC & SWE) == 0)
#define EAST_UNPRESSED   ((PINC & SWE) == SWE)

#define NORTH_PRESSED    ((PINC & SWN) == 0)
#define NORTH_UNPRESSED  ((PINC & SWN) == SWN)




#define BUFFSIZE 256


#include <avr/wdt.h>



// Function Pototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));


// Function Implementation
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();

    return;
}


typedef int bool;
#define true 1
#define false 0


enum direction {EAST,WEST,SOUTH,NORTH,CENTER};

volatile enum direction d = EAST;

volatile bool pressed = false;
volatile bool drawing = false;


void init()
{

    /* 8MHz clock, no prescaling (DS, p. 48) */
    CLKPR = (1 << CLKPCE);
    CLKPR = 0;
    wdt_init();

    init_lcd();





    PORTC |= SWW | SWS | SWE | SWN;
    PORTE |= SWC;

    /* Setup game loop timer */
    TCCR0A = _BV(WGM01);  /* CTC Mode */
    TCCR0B = _BV(CS01) | _BV(CS00); /* Prescaler: F_CPU / 64 */
    OCR0A = (uint8_t)(F_CPU / (64.0 * 1000) - 1); /* 1 ms */
    TIMSK0 |= _BV(OCIE0A);  /* Enable timer interrupt */

}


typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
    case 0:
        rgb.r = hsv.v;
        rgb.g = t;
        rgb.b = p;
        break;
    case 1:
        rgb.r = q;
        rgb.g = hsv.v;
        rgb.b = p;
        break;
    case 2:
        rgb.r = p;
        rgb.g = hsv.v;
        rgb.b = t;
        break;
    case 3:
        rgb.r = p;
        rgb.g = q;
        rgb.b = hsv.v;
        break;
    case 4:
        rgb.r = t;
        rgb.g = p;
        rgb.b = hsv.v;
        break;
    default:
        rgb.r = hsv.v;
        rgb.g = p;
        rgb.b = q;
        break;
    }

    return rgb;
}


#define soft_reset()        \
do                          \
{                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
} while(0)

void JuliaSet(double real, double imaginary, int numberOfIter, int pixelSize)
{



    //each iteration, it calculates: new = old*old + c, where c is a constant and old starts at current pixel
    double cRe, cIm;           //real and imaginary part of the constant c, determinate shape of the Julia Set
    double newRe, newIm, oldRe, oldIm;   //real and imaginary parts of new and old

    RgbColor color;

    double zoom = 1, moveX = 0, moveY = 0; //you can change these to zoom and change position
    //SDL_Color color; //the RGB color value for the pixel
    int maxIterations = numberOfIter; //after how much iterations the function should stop

    //pick some values for the constant c, this determines the shape of the Julia Set
    cRe = real;
    cIm = imaginary;

    int h = 240;
    int w = 320;

    int y,x;
    //loop through every pixel
    for (y = 0; y < h; y+=pixelSize)
        for (x = 0; x < w; x+=pixelSize)
        {


            //calculate the initial real and imaginary part of z, based on the pixel location and zoom and position values
            newRe = 1.5 * (x - w / 2) / (0.5 * zoom * w) + moveX;
            newIm = (y - h / 2) / (0.5 * zoom * h) + moveY;
            //i will represent the number of iterations
            int i;
            //start the iteration process
            for (i = 0; i < maxIterations; i++)
            {
                //remember value of previous iteration
                oldRe = newRe;
                oldIm = newIm;
                //the actual iteration, the real and imaginary part are calculated
                newRe = oldRe * oldRe - oldIm * oldIm + cRe;
                newIm = 2 * oldRe * oldIm + cIm;
                //if the point is outside the circle with radius 2: stop
                if ((newRe * newRe + newIm * newIm) > 4) break;
            }
            //use color model conversion to get rainbow palette, make brightness black if maxIterations reached
            //color = HSVtoRGB(ColorHSV(i % 256, 255, 255 * (i < maxIterations)));
            //draw the pixel

            HsvColor temp;
            temp.h = i % 256;
            temp.s = 255;
            temp.v = 255 * (i < maxIterations);
            color = HsvToRgb(temp);
            uint16_t tempCol = 0x0000;
            tempCol = ((color.r<<12)|(color.g<<6)|(color.b));

            rectangle r = {x,x+pixelSize,y,y+pixelSize};

            //screen[y][x] = tempCol;



            fill_rectangle(r,tempCol);

            //pset(x, y, color);
        }
    //make the Julia Set visible and wait to exit

}


void MandelBrot(double xCoord, double yCoord, int zoomLevel,int num, int pixelSize)
{


    //real and imaginary part of the constant c, determinate shape of the Julia Set
    double newRe, newIm, oldRe, oldIm;   //real and imaginary parts of new and old
    //double zoom = zoomLevel, moveX = -0.768171356183, moveY = -0.1022831; //you can change these to zoom and change position

    RgbColor color;

    double zoom = zoomLevel, moveX = xCoord, moveY = yCoord;
    //SDL_Color color; //the RGB color value for the pixel
    int maxIterations = num; //after how much iterations the function should stop

    double pr, pi;

    //pick some values for the constant c, this determines the shape of the Julia Set



    //Uint32* pixels = (Uint32*)img->pixels;

    //int nPixels = (img->h*(img->pitch / sizeof(unsigned int)));
    //loop through every pixel
    //int h = img->h;
    //int w = img->w;

    int h = 240;
    int w = 320;


    int y,x;
    drawing = true;
    for (y = 0; y < h ; y+=pixelSize)
    {


        for (x = 0; x < w; x+=pixelSize)
        {

            //int location = getPixel(x, y, img);
            //calculate the initial real and imaginary part of z, based on the pixel location and zoom and position values


            pr = 1.5 * (x - w / 2) / (0.5 * zoom * w) + moveX;
            pi = (y - h / 2) / (0.5 * zoom * h) + moveY;
            newRe = newIm = oldRe = oldIm = 0;
            //newRe = 1.5 * (x - w / 2) / (0.5 * zoom * w) + moveX;
            //newIm = (y - h / 2) / (0.5 * zoom * h) + moveY;
            //i will represent the number of iterations

            //start the iteration process


            int i=0;

            for (i = 0; i < maxIterations; i++)
            {
                //remember value of previous iteration
                {
                    oldRe = newRe;
                    oldIm = newIm;
                    //the actual iteration, the real and imaginary part are calculated
                    newRe = oldRe * oldRe - oldIm * oldIm + pr;
                    newIm = 2 * oldRe * oldIm + pi;
                }
                //if the point is outside the circle with radius 2: stop
                if ((newRe * newRe + newIm * newIm) > 4) break;

            }
            //use color model conversion to get rainbow palette, make brightness black if maxIterations reached




            HsvColor temp;
            temp.h = i % 256;
            temp.s = 255;
            temp.v = 255 * (i < maxIterations);
            color = HsvToRgb(temp);
            uint16_t tempCol = 0x0000;
            tempCol = ((color.r<<11)|(color.g<<5)|(color.b));

            rectangle r = {x,x+pixelSize,y,y+pixelSize};

            //screen[y][x] = tempCol;



            fill_rectangle(r,tempCol);

            //draw the pixel
            //pixels[location] = SDL_MapRGB(img->format, color.r, color.g, color.b);
        }

    }
    drawing = false;
}




void main()
{

    init();





    sei();

    display_string_xy("Welcome to MandelBrot generator",20,20);
    display_string_xy("Choose coordinates, pixel size and iteration number",10,40);
    display_string_xy("(the smaller the pixel size\the higher the iteration number the slower it is)",20,60);


    int xPos = 0, yPos =0;

    int zoomLevel = 1,pixelSize = 32;
    int numOfIter=20;

    int state = 0;

    int x[4],y[4];
    char buff[10];

    x[0]=0;
    x[1]=0;
    x[2]=0;
    x[3]=0;

    y[0]=0;
    y[1]=0;
    y[2]=0;
    y[3]=0;




    while(1)
    {


        _delay_ms(100);

        if(CENTER_PRESSED && pressed == false)
        {
            state++;
            pressed=true;
            if(state==5)
            {

                break;
            }
        }

        if(state==0)
        {

            sprintf(buff,"x=%d.%d%d%d",x[0],x[1],x[2],x[3]);
            display_string_xy(buff,20,120);







            if(EAST_PRESSED && pressed == false)
            {

                pressed = true;
                xPos++;
                if(xPos>3)
                {
                    xPos=3;
                }

            }
            if(WEST_PRESSED && pressed == false)
            {

                pressed = true;
                xPos--;
                if(xPos<0)
                {
                    xPos=0;
                }

            }
            if(SOUTH_PRESSED && pressed == false)
            {

                pressed = true;


                x[xPos]++;
                if(x[xPos]>=10)
                {

                    x[xPos]=0;
                }
            }
            if(NORTH_PRESSED && pressed == false)
            {

                pressed = true;


                x[xPos]--;
                if(x[xPos]<=-1)
                {

                    x[xPos]=9;
                }


            }
        }
        if(state==1)
        {
            sprintf(buff,"x=%d.%d%d%d",x[0],x[1],x[2],x[3]);
            display_string_xy(buff,20,120);

            sprintf(buff,"y=%d.%d%d%d",y[0],y[1],y[2],y[3]);
            display_string_xy(buff,80,120);

            if(EAST_PRESSED && pressed == false)
            {

                pressed = true;
                yPos++;
                if(yPos>3)
                {
                    yPos=3;
                }

            }
            if(WEST_PRESSED && pressed == false)
            {

                pressed = true;
                yPos--;
                if(yPos<0)
                {
                    yPos=0;
                }

            }
            if(SOUTH_PRESSED && pressed == false)
            {

                pressed = true;


                y[yPos]++;
                if(y[yPos]>=10)
                {

                    y[yPos]=0;
                }
            }
            if(NORTH_PRESSED && pressed == false)
            {

                pressed = true;


                y[yPos]--;
                if(y[yPos]<=-1)
                {

                    y[yPos]=9;
                }


            }
        }

        if(state==2)
        {
            sprintf(buff,"x=%d.%d%d%d",x[0],x[1],x[2],x[3]);
            display_string_xy(buff,20,120);

            sprintf(buff,"y=%d.%d%d%d",y[0],y[1],y[2],y[3]);
            display_string_xy(buff,80,120);

            sprintf(buff,"zoom=%d",zoomLevel);
            display_string_xy(buff,140,120);

            if(EAST_PRESSED && pressed == false)
            {

                pressed = true;


            }
            if(WEST_PRESSED && pressed == false)
            {

                pressed = true;

            }
            if(SOUTH_PRESSED && pressed == false)
            {

                pressed = true;


                zoomLevel++;
            }
            if(NORTH_PRESSED && pressed == false)
            {

                pressed = true;


                zoomLevel--;
                if(zoomLevel<=0)
                {

                    zoomLevel=1;
                }
            }
        }

        if(state==3)
        {
            sprintf(buff,"x=%d.%d%d%d",x[0],x[1],x[2],x[3]);
            display_string_xy(buff,20,120);

            sprintf(buff,"y=%d.%d%d%d",y[0],y[1],y[2],y[3]);
            display_string_xy(buff,80,120);

            sprintf(buff,"zoom=%d",zoomLevel);
            display_string_xy(buff,140,120);

            sprintf(buff,"pixel size=%d   ",pixelSize);
            display_string_xy(buff,20,140);


            if(EAST_PRESSED && pressed == false)
            {

                pressed = true;


            }
            if(WEST_PRESSED && pressed == false)
            {

                pressed = true;

            }
            if(SOUTH_PRESSED && pressed == false)
            {

                pressed = true;


                pixelSize*=2;
                if(pixelSize>32)
                {

                    pixelSize=32;
                }
            }
            if(NORTH_PRESSED && pressed == false)
            {

                pressed = true;


                pixelSize/=2;
                if(pixelSize<=0)
                {

                    pixelSize=1;
                }
            }
        }
        if(state==4)
        {
            sprintf(buff,"x=%d.%d%d%d",x[0],x[1],x[2],x[3]);
            display_string_xy(buff,20,120);

            sprintf(buff,"y=%d.%d%d%d",y[0],y[1],y[2],y[3]);
            display_string_xy(buff,80,120);

            sprintf(buff,"zoom=%d",zoomLevel);
            display_string_xy(buff,140,120);

            sprintf(buff,"pixel size=%d   ",pixelSize);
            display_string_xy(buff,20,140);

            sprintf(buff,"iterations per pixel=%d   ",numOfIter);
            display_string_xy(buff,120,140);


            if(EAST_PRESSED && pressed == false)
            {

                pressed = true;


            }
            if(WEST_PRESSED && pressed == false)
            {

                pressed = true;

            }
            if(SOUTH_PRESSED && pressed == false)
            {

                pressed = true;


                numOfIter+=2;
                if(numOfIter>=100)
                {

                    numOfIter=100;
                }
            }
            if(NORTH_PRESSED && pressed == false)
            {

                pressed = true;


                numOfIter-=2;
                if(numOfIter<=4)
                {

                    numOfIter=4;
                }
            }
        }


    }

    //display_string("STARTING");

    clear_screen();

    double positionX=x[0]+x[1]/10 + x[2]/100 + x[3]/1000;

    double positionY=y[0]+y[1]/10 + y[2]/100 + y[3]/1000;


    MandelBrot(positionX,positionY,zoomLevel,numOfIter,pixelSize);

    //goto testing;

    while(1)
    {


        if(NORTH_PRESSED && pressed == false)
        {
            pressed = true;

        }
        if(SOUTH_PRESSED&& pressed == false)
        {
            pressed = true;

        }
        if(WEST_PRESSED&& pressed == false)
        {
            pressed = true;

        }
        if(EAST_PRESSED&& pressed == false)
        {
            pressed = true;
        }

        if(CENTER_PRESSED && pressed == false)
        {
            pressed = true;
            clear_screen();
           soft_reset();
            continue;

        }

        _delay_ms(100);
    }

}


ISR( TIMER0_COMPA_vect )
{
    cli();
    if(pressed == false)
    {

        if (NORTH_PRESSED  )
        {
            d = NORTH;
            if(drawing)
            {
                soft_reset();
            }
            sei();
            return;
        }
        if (SOUTH_PRESSED )
        {
            d = SOUTH;
            if(drawing)
            {
                soft_reset();
            }
            sei();
            return;
        }
        if (EAST_PRESSED )
        {
            d = EAST;
            if(drawing)
            {
                soft_reset();
            }
            sei();
            return;
        }
        if (WEST_PRESSED )
        {
            d = WEST;
            if(drawing)
            {
                soft_reset();
            }
            sei();
            return;
        }
        if(CENTER_PRESSED)
        {
            d = CENTER;
            if(drawing)
            {
                soft_reset();
            }
            sei();
            return;

        }
    }
    else
    {
        if (NORTH_UNPRESSED && d == NORTH )
        {
            pressed = false;
            sei();
            return;
        }
        if (SOUTH_UNPRESSED && d == SOUTH)
        {
            pressed = false;
            sei();
            return;
        }
        if (EAST_UNPRESSED && d == EAST)
        {
            pressed = false;
            sei();
            return;
        }
        if (WEST_UNPRESSED && d == WEST)
        {
            pressed = false;
            sei();
            return;
        }
        if(CENTER_UNPRESSED && d == CENTER)
        {

            pressed = false;
            sei();
            return;
        }

    }
    sei();
}
