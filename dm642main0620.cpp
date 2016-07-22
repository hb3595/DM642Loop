
#include <csl.h>
#include <csl_emifa.h>
#include <csl_i2c.h>
#include <csl_gpio.h>
#include <csl_irq.h>
#include <csl_chip.h>
#include <csl_dat.h>
#include <csl_timer.h>
#include <stdio.h>
#include <fastrts62x64x.h>
#include <recip.h>

#include "iic.h"
#include "vportcap.h" 
#include "vportdis.h"
#include "sa7121h.h"
#include "sa7113h.h"
#include "dm642.h"
#include "cv.h"
#include "cxcore.h"
#include "data.h"



/*DM642��IIC�����ýṹ*/
I2C_Config DM642IIC_Config = {
    0,  
    0,  
    (20-5), 
    (20-5), 
    1,  
    0,  
    0x4ea0, 
    (75-1), 
};
//////////////////////////////////////timer(1)
TIMER_Handle hTimer0;
TIMER_Config myConfig = {

    TIMER_CTL_RMK   
    (   
        TIMER_CTL_SPND_EMUSTOP,   
        TIMER_CTL_INVINP_NO,            
        TIMER_CTL_CLKSRC_CPUOVR8,       
        TIMER_CTL_CP_PULSE,             
        TIMER_CTL_HLD_NO,            
        TIMER_CTL_GO_NO,            
                                       
        TIMER_CTL_PWID_TWO,           
                                        
        TIMER_CTL_DATOUT_1,             
        TIMER_CTL_INVOUT_YES,             
        TIMER_CTL_FUNC_TOUT            
    ),      
0xffffffff,
0x0
};
///////////////////////////////////////end
I2C_Handle hdm642i2c;
int portNumber;

extern SA7121H_ConfParams sa7121hPAL[45];
extern SA7121H_ConfParams sa7121hNTSC[45];
extern SA7113H_ConfParams sa7113hPAL[28];

/*ͼ����������*/
VP_Handle vpHchannel0;
VP_Handle vpHchannel1;
VP_Handle vpHchannel2;

/*ȷ��ͼ��Ĳ���*/
int numPixels = 720;//ÿ��720������
int numLines  = 576;//ÿ֡576�У�PAL��
///////////////////timer(2)
unsigned int timea=0;
unsigned int timeb=0;
unsigned int timec=0;
///////////////////end
static Uint8 intThreshold=90;
int numCloses = 0;
int eyex1     =0;
int eyey1     =0;
int eyex2     =0;
int eyey2     =0;   //�۾����ο������
int eyewidth  =0;
static	int a1=0;
static	int a2=0;
static	int a3=0;
////////////////////////////////////////
Uint8 *dst;//**************
CvSeq *faces;

#pragma DATA_SECTION(".histData")  //�����ں�������������������ʹ�úͶ���ǰ����
Uint8 histData[256];               //����DATA_SECTION��ָ���ռ�Ƚϴ�Ķ�ά����/// Buffer to store histogram of image

/*�ɼ�����ʾ����������ַ*/
Uint32 capYbuffer  = 0x80000000;
Uint32 capCbbuffer = 0x800675c0;
Uint32 capCrbuffer = 0x8009b0a0;

Uint32 disYbuffer  = 0x80100000;
Uint32 disCbbuffer = 0x801675c0; 
Uint32 disCrbuffer = 0x8019b0a0;

Uint32 tempYbuffer    = 0x80200000;   //��ʱ
Uint32 temp2Ybuffer   = 0x802675c0;   //��ʱ



/*ͼ���ʽ��־*/
Uint8 NTSCorPAL = 0;

extern "C" far void vectors();
extern volatile Uint32 capNewFrame;
extern volatile Uint32 disNewFrame;
extern  void VPDispIsr(void);
extern void VPCapChaAIsr(void);
void detect_and_draw( IplImage* img_samll, IplImage* img,CvMemStorage* m_storage );
CvHaarClassifierCascade* m_cascade;
int OtsuThresholding
(
	const unsigned char *restrict in,
	int ALines,int DLines,
	int APixels,int DPixels
);
void mythreshold(int ALines,int DLines,int APixels,int DPixels);
Bool IMG_sobel
(
    const unsigned char *restrict in,   /* Input image data   */
    unsigned char       *restrict out,  /* Output image data  */
    //short cols, short rows              /* Image dimensions   */
	int ALines,int DLines,
	int APixels,int DPixels
);
int eyeflags(int ALines,int DLines,int APixels,int DPixels,const unsigned char *restrict in);
void GaussSmooth
(
    const unsigned char *restrict in,   /* Input image data   */
    unsigned char       *restrict out,  /* Output image data  */
    //short cols, short rows              /* Image dimensions   */
	int ALines,int DLines,
	int APixels,int DPixels
);
Bool IMG_Retinex
(
    const unsigned char *restrict SS,   /* ��ʾͼ�� image data   */
    unsigned char       *restrict GG,  /* ƽ���� image data  */
	unsigned char       *restrict VV, /* retinex����� image data  */
	int ALines,int DLines,
	int APixels,int DPixels
);
//void do_and_flag(IplImage* img ,IplImage* imgBinary);


int main()
{
	Uint16	i;
	Uint8 addrI2C;
	////////////////////////////////////////////timer(3)
	hTimer0 = TIMER_open(TIMER_DEV0, TIMER_OPEN_RESET);
	TIMER_config(hTimer0, &myConfig);
	TIMER_start(hTimer0);
	int n=0;
	
/*-------------------------------------------------------*/
/* perform all initializations                           */
/*-------------------------------------------------------*/
	DM642_init();
//	CHIP_config(&DM642percfg);	
/*----------------------------------------------------------*/
	/*�ж�������ĳ�ʼ��*/
	//Point to the IRQ vector table
    IRQ_setVecs((void *)vectors);
    IRQ_nmiEnable();
    IRQ_globalEnable();
    IRQ_map(IRQ_EVT_VINT0, 11);
    IRQ_map(IRQ_EVT_VINT1, 12);
    IRQ_reset(IRQ_EVT_VINT0);
    IRQ_reset(IRQ_EVT_VINT1);	
    /*��һ�����ݿ���������ͨ·*/
    DAT_open(DAT_CHAANY, DAT_PRI_LOW, DAT_OPEN_2D);	

	portNumber = 1;
	vpHchannel1 = bt656_8bit_ncfd(portNumber);
    
/*----------------------------------------------------------*/	
	/*����IIC�ĳ�ʼ��*/
	hdm642i2c = I2C_open(I2C_PORT0,I2C_OPEN_RESET);
	I2C_config(hdm642i2c,&DM642IIC_Config);
/*----------------------------------------------------------*/
	GPIO_RSET(GPGC,0x0);/*��GPIO0����ΪGPINTʹ��*/
	GPIO_RSET(GPDIR,0x1);/*��GPIO0��Ϊ���*/
	GPIO_RSET(GPVAL,0x1);/*GPIO0���Ϊ�ߣ�ѡ��IIC1���ߣ�����*/

	NTSCorPAL = 0;
	addrI2C = 0x88 >>1;					      
	for(i =0; i<43; i++)
	{
		if(NTSCorPAL == 1)
		{
			_IIC_write(hdm642i2c, 
					   addrI2C,
					   (sa7121hNTSC[i].regsubaddr), 
					   (sa7121hNTSC[i].regvule));
		}
		else
		{
			_IIC_write(hdm642i2c, 
					   addrI2C,
					   (sa7121hPAL[i].regsubaddr), 
					   (sa7121hPAL[i].regvule));	
		}		
	}

	GPIO_RSET(GPVAL,0x0);

// SAA7113 ��ʼ��						   
	addrI2C = 0x4A >>1;

	for(i = 0;i<28 ;i++)
	{
				_IIC_write(hdm642i2c, 
					   addrI2C,
					   (sa7113hPAL[i].regsubaddr), 
					   (sa7113hPAL[i].regvule));

	}	

	portNumber = 0;

	const char** my_cascade=(const char**)cvAlloc( 100 );
	my_cascade[0] = ptr0;
	my_cascade[1] = ptr1;
	my_cascade[2] = ptr2;
	my_cascade[3] = ptr3;
	my_cascade[4] = ptr4;
	my_cascade[5] = ptr5;
	my_cascade[6] = ptr6;
	my_cascade[7] = ptr7;
	my_cascade[8] = ptr8;
	my_cascade[9] = ptr9;
	my_cascade[10] = ptr10;
    my_cascade[11] = ptr11;
 	my_cascade[12] = ptr12;
	my_cascade[13] = ptr13;
	my_cascade[14] = ptr14;
	my_cascade[15] = ptr15;
	my_cascade[16] = ptr16;
	my_cascade[17] = ptr17;
	my_cascade[18] = ptr18;
	my_cascade[19] = 0;

	m_cascade = (CvHaarClassifierCascade*)cvLoadHaarClassifierCascade2
	                                     (my_cascade, 19, cvSize(35,16));//���ļ���װ��ѵ���õ�����������
	
	CvMemStorage* m_storage;
	m_storage = cvCreateMemStorage(0);
	IplImage *image = cvCreateImageHeader(cvSize(720,288),IPL_DEPTH_8U,1);///720*288
	//*********/720*288,�����Ķ�ֵ��ͼƬ***start
    //IplImage *imageBinary = cvCreateImage(cvSize(720,288),IPL_DEPTH_8U,1);
	//**************************************end	
	cvSetData(image,(void *)(capYbuffer),numPixels); ///��ֵimage

	IplImage *image_small = cvCreateImage(cvSize(96,72),IPL_DEPTH_8U,1);///96*72(720/96=7.5,288/72=4)
//	IplImage *image_small = cvCreateImageHeader(cvSize(96,72),IPL_DEPTH_8U,1); 
//	cvSetData(image_small,(void *)(0x3E400),96);              


	vpHchannel0 = bt656_8bit_ncfc(portNumber);
	bt656_capture_start(vpHchannel0);
	/*�ȴ���һ֡���ݲɼ����*/
 	while(capNewFrame == 0){}
	capNewFrame =0;
//	cvResize(image,image_small,CV_INTER_NN);////����ͼƬ������������

//	cvSaveImage("image_small.bmp",image_small);
//    cvSaveImage("image.bmp", image);
//	detect_and_draw( image_small,image, m_storage );
//    cvSaveImage("image_draw.bmp",image);

	/*�����ݴ�����ʾ������������ɼ���ɵı�־*/
	/*
	for(i=0;i<numLines;i++)
	{
		//����Y������
		DAT_copy((void *)(capYbuffer + i * numPixels), 
	             (void *)(disYbuffer + i * numPixels),
	             numPixels);
	    //����Cb������
	    //DAT_copy((void *)(capCbbuffer + i * (numPixels >> 1)), 
	             //(void *)(disCbbuffer + i * (numPixels >> 1)),
	             //numPixels>>1);
		//����Cr������
	    //DAT_copy((void *)(capCrbuffer + i * (numPixels >> 1)), 
	             //(void *)(disCrbuffer + i * (numPixels >> 1)),
	             //numPixels>>1);
		
	 }*/
	//	cvReleaseImage(&image);
	/*������ʾģ��*/
	bt656_display_start(vpHchannel1);
	/*������ʾ��ʵʱѭ��*/

	for(;;)
	{
		timea=TIMER_getCount(hTimer0);
		/*���ɼ���������*/
		if((capNewFrame == 1)&&(disNewFrame == 1))
		{
			/*������װ����ʾ������������ɼ���ɵı�־*/
			capNewFrame =0;
			disNewFrame =0;
			
			for(i=0;i<numLines;i++)
			{
			    DAT_copy((void *)(capYbuffer + i * numPixels), 
			         	(void *)(tempYbuffer + i * numPixels),
			         	 numPixels);
				DAT_copy((void *)(capYbuffer + i * numPixels), 
	                    (void *)(temp2Ybuffer + i * numPixels),
	                     numPixels);
				//DAT_copy((void *)(capYbuffer + i * numPixels), 
			         	//(void *)(temp3Ybuffer + i * numPixels),
			         //	numPixels);		         	 			      
			}
			
			cvResize(image,image_small,CV_INTER_NN);///CV_INTER_NN - ���-�ھӲ岹

			//////////////////////////////////��ֵ��start
			//cvThreshold(image, imageBinary, 69, 255 , CV_THRESH_BINARY); //����OTSU�㷨�Ĳ�������
			/////////////////////////////////end
			
			detect_and_draw( image_small,image, m_storage );
			GaussSmooth
			(				
			    ( const unsigned char *)capYbuffer,
				(       unsigned char *)tempYbuffer,
				eyey1,eyey2,eyex1,eyex2
			);
			IMG_Retinex
			(
				(const unsigned char *)capYbuffer,   // ��ʾͼ�� image data   
				(unsigned char       *)tempYbuffer,  // ƽ���� image data  
				(unsigned char       *)temp2Ybuffer, // retinex����� image data 
				eyey1,eyey2,eyex1,eyex2
			);
			/*OtsuThresholding
			(
				( const unsigned char *)tempYbuffer,
				eyey1,eyey2,eyex1,eyex2
			);	
			///cvSaveImage("image_small.bmp",image_small);
			///cvSaveImage("image.bmp",image);
			mythreshold(eyey1,eyey2,eyex1,eyex2);*/
			/*IMG_sobel
			(
				( const unsigned char *)capYbuffer,
				(       unsigned char *)tempYbuffer,
				eyey1,eyey2,eyex1,eyex2
			);*/
			/*
			if(faces->total)
			{
				OtsuThresholding(
				( 
					const unsigned char *)tempYbuffer,
					eyey1,eyey2,eyex1,eyex2
				);
				
				mythreshold(0,288,0,720);
				IMG_sobel
				(
					( const unsigned char *)tempYbuffer,
					(       unsigned char *)temp2Ybuffer,
					eyey1,eyey2,eyex1,eyex2
				);
				//eyeflags(eyey1,eyey2,eyex1,eyex2,( const unsigned char *)temp2Ybuffer);
			}
			else 
			{
				//printf("close1\n");
				numCloses++;
			}*/
			//printf("a1=%d\n",a1);
			//a1=0;
			//a2=0;
			//a3=0;
//       		static int n=0;
			//n++;
			

			for(i=0;i<numLines;i++) 
			{
			
				//DAT_copy((void *)(imageBinary->imageData + i * numPixels), 
	                       //(void *)(disYbuffer+ i * numPixels),
	                       //numPixels);
				//����Y������
				DAT_copy((void *)(temp2Ybuffer + i * numPixels), 
			             (void *)(disYbuffer + i * numPixels),
			             numPixels);
		
			    //����Cb������
			    //DAT_copy((void *)(capCbbuffer + i * (numPixels >> 1)), 
			             //(void *)(disCbbuffer + i * (numPixels >> 1)),
			             //numPixels>>1);
				//����Cr������
			    //DAT_copy((void *)(capCrbuffer + i * (numPixels >> 1)), 
			             //(void *)(disCrbuffer + i * (numPixels >> 1)),
			             //numPixels>>1);
			
			}
			//////////////////////////////timer(4)
			timeb=TIMER_getCount(hTimer0);
			printf("the time spended is %d\n",timeb-timea);//��������ת���ɾ���ʱ��

			/////////////////////////////
//			capNewFrame =0;
//			disNewFrame =0;
			 ///printf("NumCloses=%d\n",numCloses);
			 //printf("Ntotal=%d\n",n);
///////////////////////////////////////////////////////////////////////////////////
/*
			if(n==150)
			{
			     if(numCloses>=70)
			     {printf("You are fatigued now!!!!!\n");}
				 else
				 {printf("You are normal now.\n");}
				 n=0;
				 numCloses=0;		 
			}
*/
		}
		
	}
}

static int second = 0;	
static CvScalar colors =  {{0,0,0}};
static CvPoint pt1,pt2;
static CvPoint pt3,pt4,pt5,pt6;
void detect_and_draw( IplImage* img_small , IplImage* img ,CvMemStorage* m_storage)//main���õ��ĺ���
{
		
	cvClearMemStorage( m_storage );         // ����ڴ�洢��
	int i;
    if(second == 0)
	{
		faces = cvHaarDetectObjects( img_small, m_cascade, m_storage,
                   1.1,2, 0,cvSize(24, 24));	                              // opencv�еĺ���(1.2��ָ��ǰ��������̵�ɨ���У��������ڵı���ϵ����������������������20%��
		                                                                     //  1��ָ���ɼ��Ŀ������ھ��ε���С������0��ָ������ʽ����������canny��Ե��⣻
		                                                                    //   ��ⴰ����С�ߴ�Ϊ24*24)��Ϊ��35,16��
																		   //    0=//CV_HAAR_DO_CANNY_PRUNING//
		/*if(faces->total==0)
		{
			numCloses++;
		}*/
		second =0;

		for( i = 0; i < (faces ? faces->total : 0); i++ )
		{
    		CvRect* r = (CvRect*)cvGetSeqElem( faces, i );			// ����������ָ����Ԫ��ָ��

		
//begin���۾�����
			pt1.x = (int)(r->x*7.5);
			pt1.y = (int)(r->y*4.5);
			pt2.x = (int)(pt1.x+r->width*10.5);//Width=10.5width,Height=2height
			pt2.y = (pt1.y+r->height*2);      //��i��eye size��
//end
			//cvRectangle( img, pt1, pt2, colors, 3, 8, 0 ); // ���Ʒ��� //	opencv�еĺ���
			
			
			// ��ȡ��⵽������λ��
			
			pt3.x=(int)(pt2.x*0.25+pt1.x*0.75)-r->width*1.3;
			pt3.y=(int)(pt2.y*0.5+pt1.y*0.5)-r->height*0.5;
			pt4.x=pt3.x+r->width*2.6;//1/4Width
			pt4.y=pt3.y+r->height+5;//1/2Height
			//eyex1=pt1.x;
			//eyex2=pt2.x;
			//eyey1=pt1.y;
			//eyey2=pt2.y;
			eyex1=pt3.x;
			eyex2=pt4.x;
			eyey1=pt3.y;
			eyey2=pt4.y;
			eyewidth=(int)(r->width*0.65);
			//cvRectangle( img, pt3, pt4, colors, 3, 8, 0 );
			
			pt5.x=pt3.x+r->width*0.65;
			pt5.y=pt3.y;
			pt6.x=pt5.x+r->width*1.3;
			pt6.y=pt4.y;
			//cvRectangle( img, pt5, pt6, colors, 3, 8, 0 );
		}
	}


	else
	{
		second = 0;
		//cvRectangle( img, pt1, pt2, colors, 3, 8, 0 ); 
		pt1.x = 0;
		pt1.y = 0;
		pt2.x = 0;
		pt2.y = 0;
				
	}

}

int OtsuThresholding
(
	const unsigned char *restrict in,
	int ALines,int DLines,int APixels,int DPixels
)
{
		int ptr,t,total,h,wB,wF ;
		float sum,sumB,varMax,mB,mF,varBetween;

		// Clear histogram data
		// Set all values to zero
		ptr = 0;
		while (ptr < 256) 
			histData[ptr++] = 0;

		// Calculate histogram and find the level with the max value
		// Note: the max level value isn't required by the Otsu method
		ptr = 0;
		int maxLevelValue = 0;/// Maximum lavel value 
		/*while (ptr < 288*720)
		{
			h = 0xFF & in[ptr];
			histData[h] ++;
			if (histData[h] > maxLevelValue) 
				maxLevelValue = histData[h];
			ptr ++;
		}*/
		///////
		while (ptr < DLines*720+DPixels)
		{
			ptr = ALines*720+APixels
			h = 0xFF & in[ptr];
			histData[h] ++;
			if (histData[h] > maxLevelValue) 
				maxLevelValue = histData[h];
			ptr ++;
		}
		///////

		// Total number of pixels
		total = 288*720;

	    sum = 0;
		for ( t=0 ; t<256 ; t++) 
			sum += t * histData[t];

		sumB = 0;
		wB = 0;
		wF = 0;

		varMax = 0;
		intThreshold = 0;

		for ( t=0 ; t<256 ; t++)
		{
			wB += histData[t];					// Weight Background
			if (wB == 0) continue;

			wF = total - wB;						// Weight Foreground
			if (wF == 0) break;

			sumB += (float) (t * histData[t]);

			mB = sumB / wB;				// Mean Background
			mF = (sum - sumB) / wF;		// Mean Foreground

			// Calculate Between Class Variance
			varBetween = (float) (wB * wF) * (mB - mF) * (mB - mF);	

			// Check if new maximum found
			if (varBetween > varMax) 
			{
				varMax = varBetween;
				intThreshold = t;
			}
		}

		// Apply threshold to create binary image
		
		//ptr = 0;
		//while (ptr < MAX_ROW*MAX_COL)
		{
			//OutputImage[ptr] = ((0xFF & ImputImage[ptr]) >= threshold) ? (char) 255 : 0;
		//	ptr ++;
		}
		
		return intThreshold;
//	}
}

void mythreshold(int ALines,int DLines,int APixels,int DPixels)
{
	int i,j;
	//������������
	for(i=ALines;i<DLines;i++)//����
	{
	    for(j=APixels;j<DPixels;j++) //������/ÿ��
	    {
	        *(Uint8 *)(temp2Ybuffer + i*numPixels + j) = *(Uint8 *)(temp2Ybuffer + i*numPixels + j)<intThreshold?0x00:0xFF;
			*(Uint8 *)(temp2Ybuffer + (i+288)*numPixels + j) = *(Uint8 *)(temp2Ybuffer + (i+288)*numPixels + j)<intThreshold?0x00:0xFF;
	    }//(Uint8 *)t��ʾǿ������ת����ת����Uint8��Ȼ�������ʾ��ת�����ָ�����	 
	}
	
	//ż��
	/*
	for(i=288+ALines;i<288+DLines;i++)//����
	{
	    for(j=APixels;j<DPixels;j++) //������/ÿ�� 
	    {		
        	*(Uint8 *)(tempYbuffer + i*numPixels + j) = *(Uint8 *)(tempYbuffer + i*numPixels + j)<intThreshold?0x00:0xFF;   
        }
	}*/		
}

Bool IMG_sobel
(
    const unsigned char *restrict in,   /* Input image data   */
    unsigned char       *restrict out,  /* Output image data  */
    //short cols, short rows              /* Image dimensions   */
	int ALines,int DLines,
	int APixels,int DPixels
)
{
    int H, O1,O2,V, i,j;
	int P;
    int i00, i01, i02;
    int i10,      i12;
    int i20, i21, i22;
    int w = 720;


    /* -------------------------------------------------------------------- */
    /*  Iterate over entire image as a single, continuous raster line.      */
    /* -------------------------------------------------------------------- */
    for(i=ALines;i<DLines-2;i++)
	{
	    for(j=APixels;j<DPixels-2;j++) 
	    {
			/* ---------------------------------------------------------------- */
			/*  Read in the required 3x3 region from the input.                 */
			/* ---------------------------------------------------------------- */
			i00=in[i*numPixels+j    ]; i01=in[i*numPixels+j    +1]; i02=in[i*numPixels+j    +2];
			i10=in[i*numPixels+j+  w];                  i12=in[i*numPixels+j+  w+2];
			i20=in[i*numPixels+j+2*w]; i21=in[i*numPixels+j+2*w+1]; i22=in[i*numPixels+j+2*w+2];
			/* ---------------------------------------------------------------- */
			/*  Apply horizontal and vertical filter masks.  The final filter   */
			/*  output is the sum of the absolute values of these filters.      */
			/* ---------------------------------------------------------------- */

			H = -   i00 - 2*i01 -   i02 +
				+   i20 + 2*i21 +   i22;//Gy

			V = -   i00         +   i02
				- 2*i10         + 2*i12
				-   i20         +   i22;//Gx

			H=abs(H);
			V=abs(V);
			O1= H + V;


			/* ---------------------------------------------------------------- */
			/*  Clamp to 8-bit range.  The output is always positive due to     */
			/*  the absolute value, so we only need to check for overflow.      */
			/* ---------------------------------------------------------------- */
			/*if (O1 > 255) O1 = 255;
			out[(numLines/2+i)*numPixels+j]=O1;*/
			if (O1 > 130) out[i*numPixels+j]=0;
            else out[i*numPixels+j]=255;
			/* ---------------------------------------------------------------- */
			/*  Store it.                                                       */
			/* ---------------------------------------------------------------- */
			//out[i + 1] = O;
			//out[i*numPixels+j]=O1;
			/////////////////////////////////////////////////////////////////////////////////////������
			
            i00=in[(numLines/2+i)*numPixels+j    ]; i01=in[(numLines/2+i)*numPixels+j    +1]; i02=in[(numLines/2+i)*numPixels+j    +2];
			i10=in[(numLines/2+i)*numPixels+j+  w];                  i12=in[(numLines/2+i)*numPixels+j+  w+2];
			i20=in[(numLines/2+i)*numPixels+j+2*w]; i21=in[(numLines/2+i)*numPixels+j+2*w+1]; i22=in[(numLines/2+i)*numPixels+j+2*w+2];
			H = -   i00 - 2*i01 -   i02 +
				+   i20 + 2*i21 +   i22;//Gy

			V = -   i00         +   i02
				- 2*i10         + 2*i12
				-   i20         +   i22;//Gx

			H=abs(H);
			V=abs(V);
			O2= H + V;
			/*if (O2 > 255) O2 = 255;
			out[(numLines/2+i)*numPixels+j]=O2;*/
			if (O2 > 130) out[(numLines/2+i)*numPixels+j]=0;
			else out[(numLines/2+i)*numPixels+j]=255;
			
		}
    }
    return TRUE;
}

int eyeflags(int ALines,int DLines,int APixels,int DPixels,const unsigned char *restrict in)
{
	for(int i=ALines; i<DLines; i++)
	{
		int n1=0;
		for(int j=APixels; j<DPixels; j++)
		{
			
			if(n1>=0&&n1<=eyewidth)
			{
				a1=a1+in[i*numPixels + j]+in[(i+288)*numPixels + j];
			}
			if(n1>eyewidth&&n1<=3*eyewidth)
			{
				a2=a2+in[i*numPixels + j]+in[(i+288)*numPixels + j];
			}
			if(n1>3*eyewidth&&n1<=4*eyewidth)
			{
				a3=a1+in[i*numPixels + j]+in[(i+288)*numPixels + j];
			}
			n1++;	
		}
	}
	
	if(abs(3*a1-(a2+a3))>=abs(3*a3-(a2+a1)))
	{a1=abs(3*a1-(a2+a3));}
    else
	{a1=abs(3*a3-(a2+a1));}

	if(a1>=100000)
	{
		printf("open\n");
	}
    else
	{
		printf("close\n");
		numCloses++;
	}
}


Bool IMG_Retinex
(
    const unsigned char *restrict SS,   /* ��ʾͼ�� image data   */
    unsigned char       *restrict GG,  /* ƽ���� image data  */
	unsigned char       *restrict VV, /* retinex����� image data  */
    //short cols, short rows              /* Image dimensions   */
	int ALines,int DLines,
	int APixels,int DPixels
)
{
    int i,j;
	float minv=abs(log(SS[ALines*numPixels+APixels])-log(GG[ALines*numPixels+APixels]));
	float maxv=abs(log(SS[ALines*numPixels+APixels+1])-log(GG[ALines*numPixels+APixels+1]));
	float Tempv,Tempvv;
    for(i=ALines;i<DLines-2;i++)
	{
	    for(j=APixels;j<DPixels-2;j++) 
	    {
			Tempv=abs(log(SS[i*numPixels+j])-log(GG[i*numPixels+j]));
			if(Tempv>=maxv) maxv=Tempv;	
			else if(Tempv<=minv) minv=Tempv;
			VV[i*numPixels+j]=Tempv;
			VV[(numLines/2+i)*numPixels+j]=Tempv;
            //VV[(numLines/2+i)*numPixels+j]=abs(log(SS[(numLines/2+i)*numPixels+j])-log(GG[(numLines/2+i)*numPixels+j])); 	
		}
    }
	for(i=ALines;i<DLines-2;i++)
	{
	    for(j=APixels;j<DPixels-2;j++) 
	    {
            Tempvv=(VV[i*numPixels+j]-minv)*255/(maxv-minv);
			VV[i*numPixels+j]=Tempvv;
			VV[(numLines/2+i)*numPixels+j]=Tempvv;
			//VV[(numLines/2+i)*numPixels+j]=abs(log(SS[(numLines/2+i)*numPixels+j])-log(GG[(numLines/2+i)*numPixels+j])); 	
		}
    }
    return TRUE;
}

void GaussSmooth
(
    const unsigned char *restrict in,   /* Input image data   */
    unsigned char       *restrict out,  /* Output image data  */
    //short cols, short rows              /* Image dimensions   */
	int ALines,int DLines,
	int APixels,int DPixels
)
{
	int i,j;
	float fTemp;
	int intTemp;	
	
	for(i=ALines;i<DLines-1;i++)
	{
	    for(j=APixels;j<DPixels-1;j++) 
	    {
	    	/*��Ļ�������н��д���*/
	    	/*�ø�˹ģ����д���*/
	    	/*fTemp = in[(numLines/2+i-1)*numPixels + (j-1)] + 
	    	        2*in[(numLines/2+i-1)*numPixels + j] + 
	    	        in[(numLines/2+i-1)*numPixels + (j+1)] +
	    		    2*in[i*numPixels + (j-1)]+ 
	    	        4*in[i*numPixels + j]+ 
	    	        2*in[i*numPixels + (j+1)] +
                    in[(numLines/2+i)*numPixels + (j-1)] + 
	    	        2*in[(numLines/2+i)*numPixels + j] + 
	    	        in[(numLines/2+i)*numPixels + (j+1)];*/
			fTemp =  
	    	        ((*(Uint8 *)(in + (numLines/2+i-1)*numPixels + j))>>2) + 
	    	        
	    		    ((*(Uint8 *)(in + i*numPixels + (j-1)))>>2) + 
	    	        1*(*(Uint8 *)(in + i*numPixels + j)) + 
	    	        ((*(Uint8 *)(in + i*numPixels + (j+1)))>>2) +
                     
	    	        ((*(Uint8 *)(in + (numLines/2+i)*numPixels + j))>>2);
	    	  		
	  		intTemp = (int)(fTemp/2 + 0.5);
	  		
	  		if(intTemp<0)
	  		{
	  			intTemp = 0;
	  		}
	  		
	  		if(intTemp>255)
	  		{
	  			intTemp = 255;
	  		}
	    	
	    	//��Ļ�������н��д���
	    	out[i*numPixels + j] = intTemp; 
	    	
	    	/*��Ļ��ż���н��д���*/
	    	/*�ø�˹ģ����д���*/
	    	/*fTemp = in[i*numPixels + (j-1)] + 
	    	        2*in[i*numPixels + j]+ 
	    	        in[i*numPixels + (j+1)]+
	    	        2*in[(i+numLines/2)*numPixels + (j-1)] + 
	    	        4*in[(i+numLines/2)*numPixels + j] + 
	    	        2*in[(i+numLines/2)*numPixels + (j+1)]+	     
	    	        in[(i+1)*numPixels + (j-1)] + 
	    	        2*in[(i+1)*numPixels + j] + 
	    	        in[(i+1)*numPixels + (j+1)];*/
            fTemp =  
	    	        ((*(Uint8 *)(in + (i-1)*numPixels + j))>>2) + 
	    	     
	    		    ((*(Uint8 *)(in + (i+numLines/2)*numPixels + (j-1)))>>2) + 
	    	        1*(*(Uint8 *)(in + (i+numLines/2)*numPixels + j)) + 
	    	        ((*(Uint8 *)(in + (i+numLines/2)*numPixels + (j+1)))>>2) +
                     
	    	        ((*(Uint8 *)(in + i*numPixels + j))>>2);					
	  		
	  		intTemp = (int)(fTemp/2 + 0.5);
	  		
	  		if(intTemp<0)
	  		{
	  			intTemp = 0;
	  		}
	  		
	  		if(intTemp>255)
	  		{
	  			intTemp = 255;
	  		}
	  		
	    	//��Ļ��ż���н��д���
	    	out[(i+numLines/2)*numPixels + j] = intTemp; 	    	
	    	
	    	
	    }
	}
			
}

////////////////////////////////////////////////////////////////////do_and_flag
/*
void do_and_flag(IplImage* img ,IplImage* imgBinary)
{
		//��������ֵ��
	//cvThreshold(img, imgBinary, normal_threshold, max_val , CV_THRESH_BINARY); //��ͨ��ֵ��
	cvThreshold(img, imgBinary, 0, 255 , CV_THRESH_OTSU); //����OTSU�㷨�Ĳ�������
	//cvThreshold(imgsrc, imgotsu, otsu_threshold, max_val , CV_THRESH_BINARY); //otsu ����Ӧ
	(Uint32 *)temp2Ybuffer=imgBinary->imageData;
}*/
//======================================end=================================
