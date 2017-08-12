#include "ebox.h"
#include "ball_on_plate.h"

//����
#include "led.h"
#include "uart_vcan.h"

#include "my_math.h"
#include "signal_stream.h"
#include <math.h>

//����
#include "Button.h"
#include "oled_i2c.h"
//����ϵͳ
#include "freertos.h"
#include "task.h"
#include "queue.h"


using namespace std;

BallOnPlate ballOnPlate;

//����
UartVscan uartVscan(&uart1);
FpsCounter fpsUI;
float fpsUItemp;
float outX, outY;

//����
const float rateUI = 10;
const float intervalUI = 1 / rateUI;
const uint32_t uiRefreshDelay = ((1000 * intervalUI + 0.5) / portTICK_RATE_MS);
Button keyL(&PB4, 1);
Button keyR(&PB1, 1);
Button keyU(&PC5, 1);
Button keyD(&PC2, 1);
Led led(&PD2, 1);
OLEDI2C oled(&i2c1);


//��������
int numIndex = 0;
int task = 0;
TicToc timerTask,timerUI;
int stage = 0;//�����״̬��0����ֹͣ��1����׼�����
bool caliSuc = true, caliBegin = false;

//·������
int abcd[4] = { 0 };
const float speedTask6=120;

void posReceiveEvent()
{
	//������Ӧ
	keyL.loop();
	keyR.loop();
	keyU.loop();
	keyD.loop();

	if (keyR.click())
	{
		numIndex++;
	}
	if (keyL.click())
	{
		numIndex--;
	}

	limit<int>(numIndex, 0, 7);

	//�ܹ�7�����֣�0������������
	//������Ӧ
	float increase = 0;
	if (keyU.click())
	{
		increase++;
	}
	if (keyD.click())
	{
		increase--;
	}
	if (keyU.pressed_for(200, 0))
	{
		increase += 1;
	}
	if (keyD.pressed_for(200, 0))
	{
		increase -= 1;
	}



	switch (numIndex)
	{
	case 0:
		stage = 0;//ֹͣ�����е�����
		task += increase;
		limit(task, 0, 7);
		break;
	case 1:
		//ѡ�������
		if (increase != 0)
		{
			//��ʼ����
			switch (task)
			{
			case 0://����0������
				if (isnan(ballOnPlate.getPosX()))//��ֹ׼��ʱ�ٶ�̫����ƽ��
				{
					ballOnPlate.setPath(4, 4, 100);
				}
				else
				{
					ballOnPlate.setPath(4, 100);
				}
				stage = 1;
				break;
			case 1://����1��2�ȶ�5
				if (isnan(ballOnPlate.getPosX()))
				{
					ballOnPlate.setPath(1, 1, 100);
				}
				else
				{
					ballOnPlate.setPath(1, 100);
				}
				stage = 1;
				break;
			case 2://����2��1~5ͣ��2<15
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.setPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(0, 4, 100);
					stage = 2;
				}
				break;
			case 3://����3��1~4ͣ��2~5ͣ��2<20
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.setPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(0, 3, 100);
					timerTask.tic();
					stage = 2;
				}
				break;
			case 4://����4��1~9ͣ��2<30
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.setPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(0, 8, 100);
					stage = 2;
				}
				
				break;
			case 5://����5��1~2~6~9ͣ��2<40
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.setPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(0, 1, 100);
					timerTask.tic();
					stage = 2;
				}
				break;
			case 6://����6���ֶ������Ⱥ󾭹�ABCD
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(abcd[0], abcd[0], speedTask6);
					}
					else
					{
						ballOnPlate.setPath(abcd[0], speedTask6);
					}
					
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(abcd[0], abcd[1], speedTask6);
					timerTask.tic();
					stage = 2;
				}
				break;
			case 7://����7��4~��5 3������~9ͣ��2
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(3, 3, speedTask6);
					}
					else
					{
						ballOnPlate.setPath(3, speedTask6);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					timerTask.tic();
					ballOnPlate.setPath(3, 11, 75);
					stage = 2;
				}
				break;
			default:
				break;
			}
		}
		break;
	case 2://A
		stage = 0;
		abcd[0] += increase;
		limit(abcd[0], 0, 8);
		break;
	case 3://B
		stage = 0;
		abcd[1] += increase;
		limit(abcd[1], 0, 8);
		break;
	case 4://C
		stage = 0;
		abcd[2] += increase;
		limit(abcd[2], 0, 8);
		break;
	case 5://D
		stage = 0;
		abcd[3] += increase;
		limit(abcd[3], 0, 8);
		break;
	case 6:
		if (increase != 0)
		{
			ballOnPlate.shutdownRasp();
		}
		break;
	case 7:
		if (increase != 0)
		{
			caliBegin = true;
		}
		break;
	default:
		break;
	}

	switch (task)
	{
	case 1:
		if (stage == 1 && !isnan(ballOnPlate.getPosX()))
		{
			timerUI.tic();
			stage = 2;
		}
		else if (stage == 2 && isnan(ballOnPlate.getPosX()))
		{
			stage = 1;
		}
		break;
	case 3:
		if (timerTask.toc() > 7000 && stage == 2)
		{
			ballOnPlate.setPath(3, 4, 100);
			stage = 3;
		}
		break;
	case 5:
		if (timerTask.toc() > 3000 && stage == 2)
		{
			ballOnPlate.setPath(1, 5, 100);
			stage = 3;
			timerTask.tic();
		}
		else if (timerTask.toc() > 4000 && stage == 3)
		{
			ballOnPlate.setPath(5, 8, 100);
			stage = 4;
		}
		break;
	case 6:
		if (timerTask.toc() > 6000 && stage == 2)
		{
			ballOnPlate.setPath(abcd[1], abcd[2], speedTask6);
			stage = 3;
			timerTask.tic();
		}
		else if (timerTask.toc() > 6000 && stage == 3)
		{
			ballOnPlate.setPath(abcd[2], abcd[3], speedTask6);
			stage = 4;
		}
		break;
	case 7:
		if (timerTask.toc() > 2000 && stage == 2)
		{
			ballOnPlate.setRoundParams(75, 0.2);
			ballOnPlate.setMode(BallOnPlate_Pos_Mode_Round);
			stage = 3;
			timerTask.tic();
		}
		break;
	default:
		break;
	}

	
	float vscan[] = {
		ballOnPlate.getPosX(),ballOnPlate.getPosY()
		,ballOnPlate.getOutX(),ballOnPlate.getOutY()
		,ballOnPlate.getTargetXRaw(),ballOnPlate.getTargetYRaw()
		//,ballOnplate.getFeedforwardX(),ballOnplate.getFeedforwardY()
		,ballOnPlate.getTargetXFiltered(),ballOnPlate.getTargetYFiltered()
	};
	uartVscan.sendOscilloscope(vscan, 8);

	//����
	outX = ballOnPlate.getOutX();
	outY = ballOnPlate.getOutY();
}


//У׼ƽ������
void caliTask(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		if (caliBegin)
		{
			caliSuc = ballOnPlate.calibrate();
			caliBegin = false;
		}
	}
}

//UI����
void uiRefresh(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		float fpsPIDtemp;
		//�ж��Ƿ���ݮ���ѹػ�
		if (!ballOnPlate.getIsPosReceiving())
		{
			fpsPIDtemp = 0;
		}
		else
		{
			fpsPIDtemp = ballOnPlate.getFps();
		}

		//��ʾ������
		oled.printf(0, 0, Oledi2c_Font_8x16, "task:");
		if (numIndex != 0)
		{
			oled.printf(40, 0, Oledi2c_Font_8x16, "%d",task);
		}
		else
		{
			oled.printf(40, 0, Oledi2c_Font_8x16_Inv, "%d", task);
		}

		//��ʾ��ʼ����
		if (numIndex != 1)
		{
			if (stage == 0)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16, "start   ");
			}
			else if (stage == 1)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16, "ready   ");
			}
			else if (stage >= 2)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16, "working ");
			}
			
		}
		else
		{
			if (stage == 0)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16_Inv, "start");
			}
			else if (stage == 1)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16_Inv, "ready");
			}
			else if (stage >= 2)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16_Inv, "working");
			}
		}

		//��ʾ4λ��
		for (int i = 0; i < 4; i++)
		{
			if (numIndex - 2 == i)
			{
				oled.printf(i * 8, 2, Oledi2c_Font_8x16_Inv, "%1d",abcd[i]+1);
			}
			else
			{
				oled.printf(i * 8, 2, Oledi2c_Font_8x16, "%1d", abcd[i] + 1);
			}
			
		}

		//��ʾ�ػ�����
		if (numIndex==6)
		{
			oled.printf(0, 4, Oledi2c_Font_6x8_Inv, "shutdown");
		}
		else
		{
			oled.printf(0, 4, Oledi2c_Font_6x8, "shutdown");
		}
		
		//��ʾУ׼����
		if (numIndex == 7)
		{
			oled.printf(72, 4, Oledi2c_Font_6x8_Inv, "cali");
		}
		else
		{
			oled.printf(72, 4, Oledi2c_Font_6x8, "cali");
		}

		if (caliSuc)
		{
			oled.printf(104, 4, Oledi2c_Font_6x8, "Suc");
		}
		else
		{
			oled.printf(104, 4, Oledi2c_Font_6x8, "Fai");
		}

		//��ʾ��ʱ
		if (stage >= 2)
		{
			oled.printf(64, 2, Oledi2c_Font_8x16, "%-8.1f", (float)timerUI.toc()/1000);
		}
		oled.printf(0, 5, Oledi2c_Font_6x8, "%-8.1f%-8.1f", ballOnPlate.getTargetXRaw(), ballOnPlate.getTargetYRaw());
		oled.printf(0, 6, Oledi2c_Font_6x8, "%-8.1f%-8.1f", ballOnPlate.getPosX(), ballOnPlate.getPosY());
		fpsUItemp = fpsUI.getFps();
		oled.printf(0, 7, Oledi2c_Font_6x8, "%-8.0f%-8.0f", fpsPIDtemp, fpsUItemp);

		vTaskDelayUntil(&xLastWakeTime, uiRefreshDelay);
	}

}

void setup()
{
	ebox_init();
	ballOnPlate.attachAfterPIDEvent(posReceiveEvent);
	ballOnPlate.begin();
	ballOnPlate.setCaliParams(-2, 6, 15, 14);
	ballOnPlate.setPath(4, 4, 100);//����


	//����
	uart1.begin(115200);
	fpsUI.begin();


	//����
	keyD.begin();
	keyL.begin();
	keyR.begin();
	keyU.begin();
	led.begin();
	oled.begin();

	set_systick_user_event_per_sec(configTICK_RATE_HZ);
	attach_systick_user_event(xPortSysTickHandler);
	xTaskCreate(uiRefresh, "uiRefresh", 512, NULL, 0, NULL);
	xTaskCreate(caliTask, "caliTask", 512, NULL, 0, NULL);
	vTaskStartScheduler();
}


int main(void)
{
	setup();


	while (1)
	{

	}

}


