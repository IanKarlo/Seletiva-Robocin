//author  Renato Sousa, 2018
#include "libs/Wrappers.h"

std::ofstream OutFile("respostas.txt");

int main(int argc, char *argv[]){
    (void)argc;
    (void)argv;
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;
    RobotVector blue(0); //Robôs azuis.
    RobotVector yellow(1); //Robôs amarelos;
    BallWrapper ball; //Bola.
    GrSim_Client grSim_client;
    long long int cnt = 0;

    while(true) {
        if (client.receive(packet)) {
            //see if the packet contains a robot detection frame:
            if (packet.has_detection()) {

                SSL_DetectionFrame detection = packet.detection();

                //Frame info:

                OutFile << "Frame " << std::to_string(cnt) << '\n';
                cnt++;

                //Ball info:
                ball.updateBall(detection);
                ball.printAll(OutFile);

                //Blue robot info:
                OutFile << "Blue robots" << '\n';
                blue.updateAll(detection);
                blue.printAll(OutFile);

                //Yellow robot info:
                OutFile << "Yellow robots" << '\n';
                yellow.updateAll(detection);
                yellow.printAll(OutFile);

            }
        }
    }

    OutFile.close();

    return 0;
}