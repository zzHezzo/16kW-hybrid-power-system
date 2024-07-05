#include "socket_tcp.h"
#include "lwip/opt.h"
//#if LWIP_SOCKET
#include <lwip/sockets.h>
#include "lwip/sys.h"
#include "lwip/api.h"
//#include "FreeRTOS.h"
#include "zzh.h"
#include "freertos_demo.h"
//#include "semphr.h"
//#include "queue.h"
/*-----------------------------------------------------------------------------------*/
#define RECV_DATA         (64)
#define LOCAL_PORT         8088

//static QueueHandle_t xQueue1 = NULL;
//static QueueHandle_t xQueue2 = NULL;
//extern volatile uint16_t valve_servo , battery_status , Electronic_Speed_Controller_relay, rectifier_relay; 
//extern SemaphoreHandle_t gpioSemaphore;
//extern SemaphoreHandle_t send_PC_Semaphore;	//���͸�PC���ź���	

static void  
tcpecho_thread(void *arg)
{
  int sock = -1,connected;
  char *recv_data;
  struct sockaddr_in server_addr,client_addr;
  socklen_t sin_size;
  int recv_data_len;
	BaseType_t err = pdFALSE;
	BaseType_t errtemp = pdFALSE;
  printf("���ض˿ں���%d\n\n",LOCAL_PORT);
 
  recv_data = (char *)pvPortMalloc(RECV_DATA);
  if (recv_data == NULL)
  {
      printf("No memory\n");
      goto __exit;
  }
 
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
      printf("Socket error\n");
      goto __exit;
  }
 
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(LOCAL_PORT);
  memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));
 
//	fcntl(sock, F_SETFL, O_NONBLOCK);//�Ƕ���ģʽ
	
  if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
  {
      printf("Unable to bind\n");
      goto __exit;
  }
 
  if (listen(sock, 5) == -1)
  {
      printf("Listen error\n");
      goto __exit;
  }
 
  while(1)
  {
    sin_size = sizeof(struct sockaddr_in);
    connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);
    printf("new client connected from (%s, %d)\n",
            inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
    {
      int flag = 1;
     
      setsockopt(connected,
                 IPPROTO_TCP,     /* set option at TCP level */
                 TCP_NODELAY,     /* name of option */
                 (void *) &flag,  /* the cast is historical cruft */
                 sizeof(int));    /* length of option value */
			
//			/*���÷�����ģʽ*/
//			int flags = fcntl(connected, F_GETFL, 0);
//			fcntl(connected, F_SETFL, flags | O_NONBLOCK);	
    }
								 
//		// ʹ�� poll
//		struct pollfd fds[1];
//		fds[0].fd = connected;
//		fds[0].events = POLLOUT;
    while(1)
    {
//			int activity = poll(fds, 1, 100); // ��ʱʱ��Ϊ 0.1 ��
//			if (activity > 0) {
//					if (fds[0].revents & POLLOUT) {
//						// �ɶ������ж�����
//						
//					}
//			}
			
			/*����Ϣ����λ��*/
			errtemp = xSemaphoreTake(send_PC_Temprature_Semaphore,0);
			err=xSemaphoreTake(send_PC_Semaphore,0);//0�ǲ����� ���Ϸ��� portMAX_DELAY					
			if(err==pdTRUE||errtemp==pdTRUE)  //��ʽ����ʱ�ĳ���
				{
					printf("��PC��������\n");
					write(connected,send_data_to_pc(),*send_data_to_pc());//д����
				}	
				
      recv_data_len = recv(connected, recv_data, RECV_DATA, MSG_DONTWAIT);
     
      if (recv_data_len <=0)  
			{
					vTaskDelay(50);
					continue;
			}
			else
			{				
				printf("�յ�������Ϊ %s \n",recv_data);//printf("��һ�±����� %d\n",recv_data[1]);
				if(recv_data[1]==0x67&&recv_data[16]==0x68)
					{
						recv_data_from_PC(recv_data);
						printf("���Ͷ�ֵ�ź���gpioSemaphore\n");
						xSemaphoreGive(gpioSemaphore);
					}
					else if(recv_data[0]=='c'&&recv_data[1]=='l'&&recv_data[2]=='o'&&recv_data[3]=='s'&&recv_data[4]=='e')
					{
						break;
					}
		//printf("recv %d len data\n",recv_data_len);
			}
							
    }
    if (connected >= 0)  
      closesocket(connected);
   
    connected = -1;
  }
__exit:
  if (sock >= 0) closesocket(sock);
  if (recv_data) free(recv_data);
}


/*-----------------------------------------------------------------------------------*/
void
tcpecho_init(void)
{
  sys_thread_new("tcpecho_thread", tcpecho_thread, NULL, 512, 4);
}
/*-----------------------------------------------------------------------------------*/




//MSG_R relay_Msg; /* ����һ���ṹ��������Ϣ���� */
///*
//*********************************************************************************************************
//*    �� �� ��: vTaskTaskUserIF
//*    ����˵��: �ӿ���Ϣ����
//*    ��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
//*    �� �� ֵ: ��
//*   �� �� ��: 1  (��ֵԽС���ȼ�Խ�ͣ������uCOS�෴)
//*********************************************************************************************************
//*/
//static void vTaskTaskUserIF(char *recv_data)
//{
//    MSG_R   *ptMsg;
//    uint8_t ucCount = 0;
//    
//        /* ��ʼ���ṹ��ָ�� */
//    ptMsg = &relay_Msg;
//    
//        /* ��ʼ������ */
//    ptMsg->battery_status = 0;
//    ptMsg->Electronic_Speed_Controller_relay = 0;
//    ptMsg->rectifier_relay = 0;        
//    
//   while(1)
//    {    

////                    ptMsg->ucMessageID++;
////                    ptMsg->ulData[0]++;;
////                    ptMsg->usData[0]++;
////						if(recv_data[1]==0x67&&recv_data[16]==0x68)
////						{
//								string_to_u16(&ptMsg->battery_status,(uint8_t*)recv_data,4);
//								string_to_u16(&ptMsg->Electronic_Speed_Controller_relay,(uint8_t*)recv_data,12);
//								string_to_u16(&ptMsg->rectifier_relay,(uint8_t*)recv_data,14);
//                    /* ʹ����Ϣ����ʵ��ָ������Ĵ��� */
//                    if(xQueueSend(xQueue2,                  /* ��Ϣ���о�� */
//                                 (void *) &ptMsg,           /* ���ͽṹ��ָ�����ptMsg�ĵ�ַ */
//                                 (TickType_t)10) != pdPASS )
//                    {
//                        /* ����ʧ�ܣ���ʹ�ȴ���10��ʱ�ӽ��� */
//                        printf("��xQueue2��������ʧ�ܣ���ʹ�ȴ���10��ʱ�ӽ���\r\n");
//                    }
//                    else
//                    {
//                        /* ���ͳɹ� */
//                        printf("��xQueue2�������ݳɹ�\r\n");                        
//                    }           
////						}
//            vTaskDelay(20);            
//        }                
//        
//}
