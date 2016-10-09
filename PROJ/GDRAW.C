/* putpixel example */
#include <graphics.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#include <dos.h>

#define PIXEL_COUNT 1000
#define DELAY_TIME  100  /* in milliseconds */

/*colors*/
#define BLACK			     0
#define BLUE			     1
#define	GREEN			     2
#define	CYAN			     3
#define	RED				     4
#define MAGENTA		     5
#define BROWN   	     6
#define GREY			     7
#define DARK_GREY	     8
#define LIGHT_BLUE     9
#define LIGHT_GREEN   10
#define LIGHT_CYAN    11
#define LIGHT_RED     12
#define LIGHT_MAGENTA 13
#define YELLOW        14
#define WHITE         15

int main()
{
	 /* request autodetection */
	 int gdriver = DETECT, gmode, errorcode;
	 int i, i2, color=BLACK, maxx, maxy, maxcolor, seed;
	 double x,y,x2,y2, startx;
	 FILE *f_ptr;

	 f_ptr = fopen("gdraw.log","r");
	 if (f_ptr == NULL)
	 {
	 outtextxy(25,40,"The log file doesn't exist!\n");
	 exit(1);
	 }

	 /* initialize graphics and local variables */
	 initgraph(&gdriver, &gmode, "");

	 /* read result of initialization */
	 errorcode = graphresult();
	 if (errorcode != grOk) {  /* an error occurred */
			printf("Graphics error: %s\n", grapherrormsg(errorcode));
			printf("Press any key to halt:");
			getch();
			exit(1);               /* terminate with an error code */
	 }

	 setbkcolor(LIGHT_BLUE);
	 cleardevice();
//	 setcolor(GREY);


	 outtextxy(25,40," DO YOU WANT TO GRAPH TACHO ?(press 'Y' to continue)");

	 if (getch() =='y')
	 {

	 /*	START PLOTTING GRAPH */
	 cleardevice();

		fscanf(f_ptr, "%lf %lf\n",&x,&y);
		i = (int) (y*50.0) +200;
		// i= (int) (y*5.0) + 200;

		startx = x;
		x = x-startx;
		x = x*2.0;
		while (!feof(f_ptr))
	 {
		fscanf(f_ptr, "%lf %lf\n",&x2,&y2);
		x2 = x2-startx;
		i2 = (int) (y2*50.0)+200;
		//i2 = (int) (y2*5.0)+200;
		x2=x2*2.0;

		line((int)x,(int)i,(int)x2,(int)i2);

		x = x2;
		i=i2;
	 }
	 line(0,200,640,200);
	 outtextxy(600,205,"time");
	 line(1,0,1,640);
	 outtextxy(2,10,"Tacho");
	 getch();
	 /* clean up */
	 closegraph();
	 /* END PLOTTING GRAPH */
	 }
	 fclose(f_ptr);
	 return 0;
}


