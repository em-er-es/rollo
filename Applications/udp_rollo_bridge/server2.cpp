/* A simple server in the internet domain using TCP
   The port number is passed as an argument */

#include "udp.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char *argv[])
{
     int sockfd, newsockfd, portno;
     socklen_t clilen;
     char buffer[256];
     struct sockaddr_in serv_addr, cli_addr;
     int n;
//     udp_client_server::udp_client udp_c("130.251.13.142",900);
//     udp_client_server::udp_client udp_c("192.168.43.54",900);
     udp_client_server::udp_client udp_c("192.168.0.120",900);
     std::cout << "WAITING FOR INCOMING CONNECTION" << std::endl;

     if (argc < 2) {
         fprintf(stderr,"ERROR, no port provided\n");
         exit(1);
     }
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
       
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0) 
          error("ERROR on accept");
	std::cout << "CLIENT CONNECTED!" << std::endl;
         while(1)
     {
     bzero(buffer,256);
     n = read(newsockfd,buffer,255);
     if (n < 0) error("ERROR reading from socket");
     else if (n==0)
	{
	std::cout << "Client disconnected, resetting server" << std::endl;
	close(newsockfd);
     	close(sockfd);
	sleep(1);
		std::cout << "WAITING FOR INCOMING CONNECTION" << std::endl;
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
       
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0) 
          error("ERROR on accept");
	std::cout << "CLIENT CONNECTED!" << std::endl;	
	}
     else{
	printf("Message received: %s\n",buffer);

		if (buffer[2]=='s')
		{
		//first byte: direction -> 0x7b = stop, 0x7c = pull, 0x7d = push, 0x7e = left, 0x7f = right)
		//second byte: velocity -> 0x50 = 0%, 0x55 = 12%, 0x56= 25%, 0x57 = 37%, 0x59 = 50%, 0x5F=62%, 0x60 = 75%, 0x61=87%, 0x62 = 100%;

		char buff_to_send[3]={0x7d,0x59,0x31}; 
		udp_c.send(buff_to_send,3);
		printf("udp_packet_Sent!! \n");
		}
		else if (buffer[2]=='l')
		{
		char buff_to_send[3]={0x7c,0x59,0x31};
		udp_c.send(buff_to_send,3);
		printf("udp_packet_Sent!! \n");
		}
		else if (buffer[2]=='f')
		{
		char buff_to_send[3]={0x7e,0x58,0x31};
		udp_c.send(buff_to_send,3);
		printf("udp_packet_Sent!! \n");
		}
		else if (buffer[2]=='g')
		{
		char buff_to_send[3]={0x7f,0x58,0x31};
		udp_c.send(buff_to_send,3);
		printf("udp_packet_Sent!! \n");
		}
		else if (buffer[2]=='o')
		{
		char buff_to_send[3]={0x7b,0x58,0x31};
		udp_c.send(buff_to_send,3);
		printf("udp_packet_Sent!! \n");
		}
		
        fflush(NULL);

	}     
     //n = write(newsockfd,"I got your message \n",18);
     //if (n < 0) error("ERROR writing to socket");
     }
     close(newsockfd);
     close(sockfd);
     
     return 0; 
}
