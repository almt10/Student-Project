#include <getopt.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <string.h>
#include <string.h>
#include <sys/socket.h>
#include <string.h>
#include <netinet/in.h>
#include <netdb.h>
#include <array>
#include <iostream>
#include <chrono>
#include <list>
#include <time.h>
#include <sys/select.h>


using namespace std;
using namespace cv;


#define RESIZE_WIDTH 84
#define RESIZE_HEIGTH 47
#define SHMSZ 5000000



pthread_mutex_t init_lock = PTHREAD_MUTEX_INITIALIZER;

class QueueLatency {
    std::list<unsigned char *> queue;
    unsigned char *buffer = NULL;

private:
    int front_element_size() {
        return *(int *)(&queue.front()[0]);
    }

public:
    void put(unsigned char *buff, int size) {
        buffer = (unsigned char *)malloc(size + sizeof(int));
        memcpy(buffer, &size, sizeof(int));
        memcpy(&buffer[sizeof(int)], buff, size);
        queue.push_back(buffer);
    }

    int canGet() {
        return queue.size();
    }

    int get(unsigned char *finalbuffer, int maxsize) {
        if (!canGet()) {
            printf("Error with the image size");
            return -1;
        }
        if (front_element_size() <= maxsize) {
            int actualSize = front_element_size();
            memcpy(finalbuffer, &queue.front()[sizeof(int)], actualSize);
            free(queue.front());
            queue.pop_front();
            return actualSize;
        }
        else {
            printf("Error with the image size");
        }
    }

    int size() {
        return queue.size();
    }
    void moveStack(unsigned char *buff, int size) {
        free(queue.front());
        queue.pop_front();
        put(buff, size);
    }

};

void error(const char *msg)
{
    perror(msg);
    exit(0);
}
int main() {

    //part in which we initialize the socket that will SEND the information
    int sockfd2, portno2, n2;
    struct sockaddr_in serv_addr2;
    struct hostent *server;
    const char *receiver = "127.0.0.1";


    portno2 = 15000;
    sockfd2 = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd2 < 0)
        error("ERROR opening socket");
    server = gethostbyname(receiver);
    if (server == NULL) {
        fprintf(stderr, "ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr2, sizeof(serv_addr2));
    serv_addr2.sin_family = AF_INET;
    bcopy(server->h_addr, (char *) &serv_addr2.sin_addr.s_addr, server->h_length);
    serv_addr2.sin_port = htons(portno2);
    if (connect(sockfd2, (struct sockaddr *) &serv_addr2, sizeof(serv_addr2)) < 0)
        error("ERROR connecting");


    char *sendBufferSize = NULL;
    char *sendBufferSize2 = NULL;
    socklen_t len;
    getsockopt(sockfd2, SOL_SOCKET, SO_RCVBUF, &sendBufferSize, &len);
    sendBufferSize="2000000";
    int g = setsockopt(sockfd2, SOL_SOCKET, SO_RCVBUF, sendBufferSize, sizeof(int));
    getsockopt(sockfd2, SOL_SOCKET, SO_RCVBUF, &sendBufferSize2, &len);
    //part in which we initialize the socket that will RECEIVE the information
    int sockfd, newsockfd, portno;
    socklen_t clilen;
    unsigned char *recvbuffer = NULL;
    recvbuffer = (unsigned char *) malloc(40000000);
    struct sockaddr_in serv_addr, cli_addr;
    int n;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = 27015;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR on binding");
    printf("Estoy escuchando....\n");
    listen(sockfd, 5); //5 means the number
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    printf("Conexion aceptada....\n");
    if (newsockfd < 0)
        error("ERROR on accept");

    //part in which we initialize the socket that will SEND the FPS to the other localhost program
    int sockfdFPS, portnoFPS, nFPS=0;
    struct sockaddr_in serv_addrFPS;
    struct hostent *serverFPS;
    const char *receiverFPS = "127.0.0.1";


    portnoFPS = 18000;
    sockfdFPS = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfdFPS < 0)
        error("ERROR opening socket");
    serverFPS = gethostbyname(receiverFPS);
    if (serverFPS == NULL) {
        fprintf(stderr, "ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addrFPS, sizeof(serv_addrFPS));
    serv_addrFPS.sin_family = AF_INET;
    bcopy(serverFPS->h_addr, (char *) &serv_addrFPS.sin_addr.s_addr, serverFPS->h_length);
    serv_addrFPS.sin_port = htons(portnoFPS);
    if (connect(sockfdFPS, (struct sockaddr *) &serv_addrFPS, sizeof(serv_addrFPS)) < 0)
        error("ERROR connecting FPS");


    //we declare the variables related with the shared memory
    char cc;
    int shmidL, shmidR, shmidCon;
    key_t keyL, keyR, keyCon;
    char *shmL,*shmR;
    char *sL, *sR;
    char *shmCon, *sCon;


    keyL = 5678;
    keyR= 1567;

    if ((shmidL = shmget(keyL, SHMSZ, IPC_CREAT | 0666)) < 0) {
        perror("shmgetL");
        exit(1);
    }
    if ((shmidR = shmget(keyR, SHMSZ, IPC_CREAT | 0666)) < 0) {
        perror("shmgetR");
        exit(1);
    }


    if((shmL = (char *) shmat(shmidL, NULL, 0)) == (char *) -1){
        perror("shmatL");
        exit(1);
    }
    sL=shmL;

    if((shmR = (char *) shmat(shmidR, NULL, 0)) == (char *) -1){
        perror("shmat");
        exit(1);
    }
    sR=shmR;

    keyCon = 2000;
    if((shmidCon = shmget(keyCon, SHMSZ, IPC_CREAT | 0666))< 0){
        perror("shmgetR");
        exit(1);
    }

    if((shmCon = (char *) shmat(shmidCon, NULL, 0)) == (char *) -1){
        perror("shmat");
        exit(1);
    }
    sCon=shmCon;
    sCon[0] = 'N';
    //we declare some variables used to retrieve the data from the socket
    char side;
    int size=0;
    int ImageSize=0;
    int c=0;
    char readyMemory = 'Y';
    int bytesToSend = 0;
    int length_toread=0;
    bool both = false;
    bool sendBoth = false;
    char retrieveSide = 'L';
    bool send =false;
    int seqNumber = 1;
    bool SharedMemoryReady = true;

    unsigned char *bufferL = NULL;
    unsigned char *bufferR = NULL;
    unsigned char *info = NULL;
    unsigned char *config = NULL;
    unsigned char *ready = NULL;
    ready = (unsigned char *) malloc(20);
    config = (unsigned char *) malloc(20);
    bufferL = (unsigned char *) malloc(150000000);
    bufferR = (unsigned char *) malloc(150000000);
    info = (unsigned char *) malloc(150000000);
    unsigned char *sharedMemory = (unsigned char*)"smR";

    //we get some useful information from the socket
    int zedWidth = 0;
    int zedHeight = 0;
    int focal = 0;
    int resizeWidth = RESIZE_WIDTH;
    int resizeHeigth = RESIZE_HEIGTH;
    n = read(newsockfd, recvbuffer, 13);
    if (n < 0) error("ERROR reading from socket");
    if (n > 0) {
        if (recvbuffer[0] != 'A') error("ERROR reading from socket the initial label 'A' ");
        else {
            zedWidth = *(int *) &recvbuffer[1];
            zedHeight = *(int *) &recvbuffer[5];
            focal = *(int *) &recvbuffer[9];
            //memcpy(&recvbuffer[13], &resizeHeigth, sizeof(int));
            //memcpy(&recvbuffer[17], &resizeWidth, sizeof(int));
            //n2 = write(sockfd2, recvbuffer, 21);
            n2 = write(sockfd2, recvbuffer, 13);
            if (n2 < 0) error("ERROR writing to socket");
        }
    }


    //some other variables for measure the FPS we have while sending and receiving
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds;

    std::chrono::time_point<std::chrono::system_clock> start2, end2;
    start2 = std::chrono::system_clock::now();
    end2 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds2;

    int receivedFPS =0;
    int sendFPS= 0;
    int recvc = 0;
    int sendc = 0;
    int v = 1;


    //some variables to make a select in the socket to localhost
    fd_set readfds;
    fd_set original_socket;
    fd_set original_stdin;

    FD_ZERO(&readfds);
    FD_ZERO(&original_socket);
    FD_ZERO(&original_stdin);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_SET(sockfd2, &original_socket);
    FD_SET(0, &original_stdin);
    FD_SET(sockfd2, &readfds);

    char encode;
    int categoryR=0;
    int categoryL=0;
    int sizeL=0;
    int sizeR=0;


    //variables for measuring the latency
    int f=0;
    std::chrono::time_point<std::chrono::system_clock> startSending, endSending;
    std::chrono::duration<double> elapsed_seconds_Sending;
    double sending[10000];
    std::ofstream fs("sendingLatency.txt");


    std::chrono::time_point<std::chrono::system_clock> startReading, endReading;
    std::chrono::duration<double> elapsed_seconds_Reading;
    double reading[10000];
    double fps[10000];
    std::ofstream fs2("readingLatency.txt");
    std::ofstream fs3("fps.txt");
    int o = 0;


    //main loop
    int i = 1;
    while (i < 2) {
        //we control the FPS we receive
        elapsed_seconds= end-start;
        if( (elapsed_seconds.count() * 1000)> 1000){
            receivedFPS = recvc;
            sendFPS = sendc ;
            sendc=0;
            recvc=0;
            start = end;

            if(o < 1000){
                fps[o] = receivedFPS;
                fs3 << fps[o] << endl;
            }
            o++;
            memcpy(config, &receivedFPS, sizeof(int));
            nFPS = write(sockfdFPS, config, sizeof(int));

            printf("\nFPS received from ZED PC(extra socket too): %d ..... sent to localhost: %d",receivedFPS, sendFPS);
        }


        if(nFPS<0){
            printf("\nError when sending the receivedFPS");
        }

        //we test if the socket where we receive the information has some data
        n = read(newsockfd, recvbuffer, 2);
        if (n < 0) error("ERROR reading from socket");
        if (n > 0) {
            if ((recvbuffer[0] != 'a') || (recvbuffer[1] != 'a'))error("ERROR reading from socket the initial labels");
            else {
                startReading = std::chrono::system_clock::now();
                n = read(newsockfd, recvbuffer, 6);
                if (n < 0) error("ERROR reading from socket");
                if (n > 0) {
                    side = recvbuffer[0];
                    encode = recvbuffer[1];
                    size = *(int *) &recvbuffer[2];
                    if(encode == 'Y'){
                        n = read(newsockfd, recvbuffer, 4);
                        if (n < 0) error("ERROR reading from socket");
                        if (n > 0) {
                            if(side == 'L'){
                                categoryL = *(int *)recvbuffer;
                                sizeL = size;
                            }
                            else if(side == 'R'){
                                categoryR = *(int *)recvbuffer;
                                sizeR = size;
                            }
                            else{
                                printf("Errors when getting the capacity\n");
                            }
                        }
                    }

                    length_toread = size;
                    c = 0;
                    while (length_toread > 0) {
                        int x = min(400000, length_toread);
                        n = read(newsockfd, recvbuffer, min(400000, length_toread));
                        if (n < 0) error("ERROR reading from socket the image data");
                        if (n > 0) {
                            if (side == 'L') {
                                memcpy(&bufferL[c], recvbuffer, n);
                                length_toread -= n;
                                c += n;
                            } else if (side == 'R') {
                                memcpy(&bufferR[c], recvbuffer, n);
                                c += n;
                                length_toread -= n;
                                if (c == size) {
                                    both = true;
                                    recvc++;
                                }
                            } else {
                                error("ERROR copying the data in the buffers");
                            }

                        }
                    }
                }
            }
        }

        endReading = std::chrono::system_clock::now();
        elapsed_seconds_Reading = endReading - startReading;
        if(f < 10000){
            reading[f] = elapsed_seconds_Reading.count() * 1000;
            fs2 << reading[f] << endl;
        }

        startSending = std::chrono::system_clock::now();
        if(side == 'L'){
            memcpy(info, &seqNumber, sizeof(int));
            info[4] = 'a';
            info[5] = 'a';
            info[6] = side;
            info[7] = encode;
            if(encode == 'Y') {
                memcpy(&info[8], &sizeL, sizeof(int));
                memcpy(&info[12], &categoryL, sizeof(int));
                memcpy(&info[16], bufferL, sizeL);
                bytesToSend = sizeL + 16;
            }
            else if(encode =='N'){
                memcpy(&info[8], &size, sizeof(int));
                memcpy(&info[12], bufferL, size);
                bytesToSend = size + 12;
            }

            pthread_mutex_lock(&(init_lock));
            memcpy(sL, info, bytesToSend);
            pthread_mutex_unlock(&(init_lock));

        }
        else if(side == 'R'){
            memcpy(info, &seqNumber, sizeof(int));
            info[4] = 'a';
            info[5] = 'a';
            info[6] = side;
            info[7] = encode;
            if(encode == 'Y') {
                memcpy(&info[8], &sizeR, sizeof(int));
                memcpy(&info[12], &categoryR, sizeof(int));
                memcpy(&info[16], bufferR, sizeR);
                bytesToSend = sizeR + 16;
            }
            else if(encode =='N'){
                memcpy(&info[8], &size, sizeof(int));
                memcpy(&info[12], bufferR, size);
                bytesToSend = size + 12;
            }
            pthread_mutex_lock(&(init_lock));
            memcpy(sR, info, bytesToSend);
            pthread_mutex_unlock(&(init_lock));

            endSending = std::chrono::system_clock::now();
            elapsed_seconds_Sending = endSending - startSending;
            if(f < 10000){
                sending[f] = elapsed_seconds_Sending.count() * 1000;
                fs << sending[f] << endl;
            }
            f++;
            n2 = write(sockfd2, sharedMemory, 3);
            if(n2<0)
                error("ERROR sending label of shared memory ready");

            seqNumber++;
            sendc++;
            end = std::chrono::system_clock::now();
        }


    }


    //we send this data
    /*if(send) {
        n2 = write(sockfd2, sharedMemory, 3);
        if (n2 < 0)
            error("ERROR writing to socket");
        if (n2 > 0) {
            //printf("Data sent through the socket : %s\n", sharedMemory);
            send = false;
            end = std::chrono::system_clock::now();
            sendc++;
        }
    }*/




    return 0;
}