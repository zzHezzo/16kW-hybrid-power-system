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
//extern SemaphoreHandle_t send_PC_Semaphore;	//发送给PC的信号量	

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
  printf("本地端口号是%d\n\n",LOCAL_PORT);
 
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
 
//	fcntl(sock, F_SETFL, O_NONBLOCK);//非堵塞模式
	
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
			
//			/*设置非阻塞模式*/
//			int flags = fcntl(connected, F_GETFL, 0);
//			fcntl(connected, F_SETFL, flags | O_NONBLOCK);	
    }
								 
//		// 使用 poll
//		struct pollfd fds[1];
//		fds[0].fd = connected;
//		fds[0].events = POLLOUT;
    while(1)
    {
//			int activity = poll(fds, 1, 100); // 超时时间为 0.1 秒
//			if (activity > 0) {
//					if (fds[0].revents & POLLOUT) {
//						// 可读，进行读操作
//						
//					}
//			}
			
			/*发消息给上位机*/
			errtemp = xSemaphoreTake(send_PC_Temprature_Semaphore,0);
			err=xSemaphoreTake(send_PC_Semaphore,0);//0是不阻塞 马上返回 portMAX_DELAY					
			if(err==pdTRUE||errtemp==pdTRUE)  //正式工作时改成与
				{
					printf("向PC发送数据\n");
					write(connected,send_data_to_pc(),*send_data_to_pc());//写数据
				}	
				
      recv_data_len = recv(connected, recv_data, RECV_DATA, MSG_DONTWAIT);
     
      if (recv_data_len <=0)  
			{
					vTaskDelay(50);
					continue;
			}
			else
			{				
				printf("收到的数据为 %s \n",recv_data);//printf("第一下标数据 %d\n",recv_data[1]);
				if(recv_data[1]==0x67&&recv_data[16]==0x68)
					{
						recv_data_from_PC(recv_data);
						printf("发送二值信号量gpioSemaphore\n");
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




//MSG_R relay_Msg; /* 定义一个结构体用于消息队列 */
///*
//*********************************************************************************************************
//*    函 数 名: vTaskTaskUserIF
//*    功能说明: 接口消息处理。
//*    形    参: pvParameters 是在创建该任务时传递的形参
//*    返 回 值: 无
//*   优 先 级: 1  (数值越小优先级越低，这个跟uCOS相反)
//*********************************************************************************************************
//*/
//static void vTaskTaskUserIF(char *recv_data)
//{
//    MSG_R   *ptMsg;
//    uint8_t ucCount = 0;
//    
//        /* 初始化结构体指针 */
//    ptMsg = &relay_Msg;
//    
//        /* 初始化数组 */
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
//                    /* 使用消息队列实现指针变量的传递 */
//                    if(xQueueSend(xQueue2,                  /* 消息队列句柄 */
//                                 (void *) &ptMsg,           /* 发送结构体指针变量ptMsg的地址 */
//                                 (TickType_t)10) != pdPASS )
//                    {
//                        /* 发送失败，即使等待了10个时钟节拍 */
//                        printf("向xQueue2发送数据失败，即使等待了10个时钟节拍\r\n");
//                    }
//                    else
//                    {
//                        /* 发送成功 */
//                        printf("向xQueue2发送数据成功\r\n");                        
//                    }           
////						}
//            vTaskDelay(20);            
//        }                
//        
//}
